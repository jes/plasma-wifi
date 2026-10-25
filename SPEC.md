# Plasma WiFi

I am building a CNC plasma cutter that uses a cheap HF-start plasma unit. Normally I would use UGS to control Grbl on an Arduino,
but in my experiments I found that the HF-start intermittently interrupts the USB communication even though the actual motion control
continues functioning. I have not been able to solve this problem.

So instead I am working on a new controller using FluidNC on an Arduino Nano ESP32, and the plan is to control it over WiFi.
I have experimented with this and found that lighting the plasma arc 50 or more times still does not interrupt motion control,
and now that we don't need a continuous USB connection the job can just run to completion.

I am however a little bit skeptical of streaming g-code over WiFi, I'd be much happier uploading the full g-code file to
FluidNC and having it run it locally. I can't be bothered installing an SD card module yet, so I am uploading g-code files
to LocalFS.

The FluidNC WebUI totally sucks, so that's where you come in.

I want you to make me a g-code sender for FluidNC that works by sending the g-code file to the LocalFS on the FluidNC.

This must be a Linux desktop application. The toolkit is up to you, but deployment should be as close as possible to dropping a
single binary onto an old laptop and running it. Prefer static linking where practical; avoid requiring libraries to be installed
or depending on a recent host glibc. The target FluidNC version is 4.1.1.

My philosophy of hobby CNC machines is that limit switches are pure cost and liability, so I don't have limit switches and won't need
any homing support.

I want to load up the plasma-wifi program and have it automatically try to connect to 192.168.0.1:23 and speak to FluidNC over telnet
or whatever it is. It should have some general "state" indicator that can be things like "disconnected", "idle", "running", "paused",
"alarm", whatever the FluidNC states are. This should be a large-ish label somewhere near top left and colour coded.

And then we need something that lets me load up a gcode file locally and send it to "plasma.nc" on the LocalFS. Delete the existing
"plasma.nc" if any, our convention will be that "plasma.nc" is owned by the plasma-wifi program.
Opening a file should automatically preprocess and upload it; loading and uploading are one user action, with visible progress,
success or failure feedback. There should not be a separate required Upload step. Uploading must not start the job: Run is explicit.
Do not replace the controller's job file while it is running or paused. Run should only be available once the current file has been
successfully uploaded and verified. If disconnected, show that the file is awaiting upload rather than implying it is ready to run.

It can do some preprocessing to strip out comments, unnecessary spaces, etc. to make the program as short as possible but it *must not*
inadvertently change the behaviour of the program because nobody wants to have to debug that.
The deliberate exception is rounding positional words
(X/Y/Z/I/J/K/R) to no more than three decimal places in the file’s units. Strip trailing zeroes and unnecessary leading
zeroes on G codes: emit G0, G1, G2, etc. Preview and timing must use the same rounded coordinates as the upload.
Accept FreeCAD’s unused K0 on XY arcs, matching FluidNC 4.1.1; this is not a drilling-cycle command.

The CAM package is not yet decided. Compatibility with FreeCAD CAM's Grbl postprocessor is an expected baseline. Support G2/G3 arcs
as well as linear moves, and track modal state correctly for processing, preview and timing. Most files will use millimetres and
absolute moves, but do not silently assume those modes: handle G20/G21 and G90/G91. If a construct cannot be processed or estimated
correctly, report that limitation clearly rather than silently changing its meaning or showing a misleading preview or estimate.

Pierce delay belongs in the CAM program because it depends on material thickness. Preserve CAM-authored G4 dwells
unchanged and include them in the uploaded-code view, line mapping and time estimate. The sender must not configure or
insert pierce delays, or split combined torch ignition/motion blocks to add them.

And then I need controls for feed override. I don't need spindle speed control at all as it is a plasma cutter.

And I need jog controls. Need to be able to set jog speed obviously. There's no Z axis btw so I only need X/Y control.
The Z configuration in config.yaml and the breakout board's Z step/direction pins are intentional, but there is currently no
Z stepper or driver fitted. Leave that controller configuration in place; the sender only needs X/Y support. Z support can be
added later if the hardware is installed.
Include keyboard jogging with the arrow keys. Arrow keys must still work normally when editing text or a numeric field;
keyboard jogging must not trigger from those fields. Stop continuous keyboard jogging when the key is released or the application
loses focus, and do not allow jogging during a running or paused job.

And I want a 2d view of the toolpath. Need to be able to set coordinates. I don't bother using G43 or whatever it is to define different
coordinate systems, I just want to manually jog the toolhead to the origin point of my work and then set X0 and Y0. I also do want to be
able to manually type these in. So add something like a DRO where I can either click X0 and Y0 at the left, or else click on the number
and type in a new one. And a "/2" button to halve it for centering on things, I'm sure that makes sense. Editing the DRO needs to change
the coordinate system and not move the toolhead.

And I want a textarea-type view of the gcode. Should just be a scrollable view that has all the gcode on it, maybe a button to toggle between
the original file and the processed-down one that takes up less space on the LocalFS.
Underneath this give some status bar stats like the total number of lines and bytes for both the full one and the stripped one, and how
many bytes are still free on the LocalFS if it's possible to get that.

While the job is running I want you to keep track of how long it has been running for and how long you expect it to take,
based on distances and feed rates in the gcode, if that makes sense, not just on the number of lines. So for each line of gcode
you can work out how many seconds that line will take to run if acceleration is infinite, based on the distance and feedrate,
and then also you can internally annotate each line with the cumulative time taken to get to that point so far, or cumulative time remaining,
whatever makes more sense, and then you know how long it's been running, know how long it is expected to take in total, and
can efficiently tell me at any given point how long remains in the program. Don't count it as running time while it is paused. Keep an
active coefficient for how much the acceleration etc. is ruining our estimate, so if we thought it would take 150 seconds to get to the
current point but it actually took 100 seconds then the coefficient is 0.66 or whatever and we can multiply all our other estimates by that.
Also account for feed override in this. If feed override was set to 200% then of course it will have completed in half the time (in expectation)
so take that into account, if we then reduce feed override to 100% then it should double our estimate of time remaining if that makes sense.
Maybe need to keep track of an aggregate feed override setting for all lines so far, idk how to do that but I bet you do. For rapid moves
I don't mind if you ask FluidNC to tell you the rapid speed (once! at startup! not every time) or hardcode something believable.

I want a display saying how long it's been running for so far and how long is expected to remain based on the remaining lines of gcode and the
coefficient.

The textarea-type view of gcode should highlight the currently active line if possible while running or paused, and should have a tickbox
for auto-scroll which will scroll it so the currently active-line is centered in the view. (If it's the first few lines then obviously don't
scroll so that the top line is not at the top, don't mind so much for the last line).

And obviously I'll want run, pause, stop, whatever buttons.

Pause must hold motion and turn off the torch, remembering whether it was lit. On Resume, if the torch was lit before the pause,
light it again and resume motion once torch restoration is confirmed. If it was off, resume without lighting it.
The sender adds no delay on Resume. This must work while executing the uploaded LocalFS file; establish the FluidNC 4.1.1 mechanism for doing it
before implementing this behavior. Stop must abort the job and turn off the torch; a subsequent Run starts from the beginning.
CAM-authored dwell time must be accounted for in elapsed and remaining time; time spent paused
is excluded. Feed override must not scale dwell time.

https://github.com/bdring/FluidNC/issues/330 this github issue may be useful, we will need to enable use_line_numbers:true in config.yaml.
I have put my active config.yaml in this dir, you can make changes to it as required and I will install it using the WebUI. And just preprocess
the gcode to set the N numbers, no big deal.

Try to auto-reconnect to the socket if disconnected.

Check file size after uploading, and potentially verify file contents if quick and easy to do.

## Second iteration

Use the system file chooser rather than a custom in-app browser, while retaining portable static deployment. Give the
window a taskbar/toolbar icon. Separate the application log from a FluidNC console, and allow typed commands in that
console with command history. Console pause/resume must preserve the torch behavior above.

DRO and feed override feedback should update promptly, using regular controller reports and refreshes after changes.
Give feed moves a more prominent preview color than rapids. Clear the current alarm message after the controller leaves
Alarm; retain historical alarm messages in the console. A normal Stop should hold first, then reset after motion stops,
to avoid FluidNC’s reset-during-motion ALARM:3. Do not automatically unlock genuine alarms.

Add a physical convex-hull trace of the loaded job, like UGS Platform: follow its closed XY outline at the current jog
speed with the torch off, as a sanity check of physical travel before cutting. Show the hull in the preview. Include the
job’s rapid travel and arc extrema. Return to the starting position after the outline. Allow cancellation, and prohibit
tracing during a running or paused job, uploading, or manual jogging. Tracing must not execute the file or ignite the torch.
