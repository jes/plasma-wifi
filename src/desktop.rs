//! Pure-Rust X11 integration. No libX11, OpenGL, GTK, Qt or host libc is loaded.
use anyhow::{Context, Result, ensure};
use egui::{Event as InputEvent, Key, Modifiers, PointerButton, Pos2, RawInput, Rect, Vec2};
use egui_software_backend::{BufferMutRef, ColorFieldOrder, EguiSoftwareRender};
use std::{
    collections::HashSet,
    thread,
    time::{Duration, Instant},
};
use x11rb::{
    COPY_DEPTH_FROM_PARENT,
    connection::{Connection, RequestConnection},
    protocol::{Event, xproto::*},
    wrapper::ConnectionExt as _,
};

pub fn set_ui_scale(ctx: &egui::Context, scale: f32) {
    // egui applies the zoom on the next pass. The backend needs that same
    // pending scale when translating physical pointer coordinates to points.
    ctx.data_mut(|data| data.insert_temp(egui::Id::new("ui_scale"), scale));
    ctx.set_zoom_factor(scale);
}

pub fn run(mut draw: impl FnMut(&mut egui::Ui)) -> Result<()> {
    let (conn, screen_id) = x11rb::connect(None)
        .context("Cannot connect to X11. Run from an X11 desktop or enable XWayland.")?;
    let screen = &conn.setup().roots[screen_id];
    let format = conn
        .setup()
        .pixmap_formats
        .iter()
        .find(|f| f.depth == screen.root_depth)
        .context("No X11 pixel format")?;
    ensure!(
        format.bits_per_pixel == 32 && conn.setup().image_byte_order == ImageOrder::LSB_FIRST,
        "This build requires a little-endian 24/32-bit X11 display"
    );
    let visual = screen
        .allowed_depths
        .iter()
        .flat_map(|d| &d.visuals)
        .find(|v| v.visual_id == screen.root_visual)
        .unwrap();
    ensure!(
        visual.red_mask == 0xff0000 && visual.green_mask == 0xff00 && visual.blue_mask == 0xff,
        "Unsupported X11 color format"
    );
    let window = conn.generate_id()?;
    let gc = conn.generate_id()?;
    let (mut width, mut height) = (
        1180u16.min(screen.width_in_pixels),
        1000u16.min(screen.height_in_pixels.saturating_sub(70)),
    );
    conn.create_window(
        COPY_DEPTH_FROM_PARENT,
        window,
        screen.root,
        0,
        0,
        width,
        height,
        0,
        WindowClass::INPUT_OUTPUT,
        0,
        &CreateWindowAux::new()
            .background_pixel(0x121820)
            .event_mask(
                EventMask::EXPOSURE
                    | EventMask::STRUCTURE_NOTIFY
                    | EventMask::KEY_PRESS
                    | EventMask::KEY_RELEASE
                    | EventMask::BUTTON_PRESS
                    | EventMask::BUTTON_RELEASE
                    | EventMask::POINTER_MOTION
                    | EventMask::LEAVE_WINDOW
                    | EventMask::FOCUS_CHANGE,
            ),
    )?;
    conn.create_gc(gc, window, &CreateGCAux::new())?;
    conn.change_property8(
        PropMode::REPLACE,
        window,
        AtomEnum::WM_NAME,
        AtomEnum::STRING,
        b"Plasma WiFi",
    )?;
    conn.change_property8(
        PropMode::REPLACE,
        window,
        AtomEnum::WM_CLASS,
        AtomEnum::STRING,
        b"plasma-wifi\0PlasmaWiFi\0",
    )?;
    let atom = |name: &[u8]| -> Result<u32> { Ok(conn.intern_atom(false, name)?.reply()?.atom) };
    let protocols = atom(b"WM_PROTOCOLS")?;
    let delete = atom(b"WM_DELETE_WINDOW")?;
    let clipboard = atom(b"CLIPBOARD")?;
    let utf8 = atom(b"UTF8_STRING")?;
    let targets = atom(b"TARGETS")?;
    let paste_property = atom(b"PLASMA_WIFI_PASTE")?;
    let mut icons = Vec::new();
    for size in [32, 64] {
        icons.extend(window_icon(size));
    }
    conn.change_property32(
        PropMode::REPLACE,
        window,
        atom(b"_NET_WM_ICON")?,
        AtomEnum::CARDINAL,
        &icons,
    )?;
    conn.change_property32(
        PropMode::REPLACE,
        window,
        protocols,
        AtomEnum::ATOM,
        &[delete],
    )?;
    conn.map_window(window)?;
    conn.flush()?;
    let setup = conn.setup();
    let mapping = conn
        .get_keyboard_mapping(setup.min_keycode, setup.max_keycode - setup.min_keycode + 1)?
        .reply()?;
    let ctx = egui::Context::default();
    ctx.data_mut(|data| data.insert_temp(egui::Id::new("native_window"), window));
    ctx.set_pixels_per_point(1.0);
    let mut renderer = EguiSoftwareRender::new(ColorFieldOrder::Bgra);
    let mut pixels = Vec::<[u8; 4]>::new();
    let mut clipboard_text = String::new();
    let mut focused = true;
    let mut current_modifiers = Modifiers::NONE;
    let mut pointer_position: Option<Pos2> = None;
    let mut pressed = HashSet::new();
    let mut last_key_release = None;
    let start = Instant::now();
    let mut quit = false;
    while !quit {
        let frame_start = Instant::now();
        let pixels_per_point = ctx
            .data(|data| data.get_temp::<f32>(egui::Id::new("ui_scale")))
            .unwrap_or(1.0);
        let mut input = RawInput {
            screen_rect: Some(Rect::from_min_size(
                Pos2::ZERO,
                Vec2::new(width as f32, height as f32) / pixels_per_point,
            )),
            time: Some(start.elapsed().as_secs_f64()),
            focused,
            modifiers: current_modifiers,
            ..Default::default()
        };
        while let Some(event) = conn.poll_for_event()? {
            match event {
                Event::ClientMessage(e)
                    if e.type_ == protocols && e.data.as_data32()[0] == delete =>
                {
                    quit = true;
                }
                Event::DestroyNotify(_) => {
                    quit = true;
                }
                Event::ConfigureNotify(e) => {
                    width = e.width.max(1);
                    height = e.height.max(1);
                }
                Event::FocusIn(_) => {
                    focused = true;
                    input.events.push(InputEvent::WindowFocused(true));
                }
                Event::FocusOut(_) => {
                    focused = false;
                    input.modifiers = Modifiers::NONE;
                    for key in pressed.drain() {
                        input.events.push(InputEvent::Key {
                            key,
                            physical_key: Some(key),
                            pressed: false,
                            repeat: false,
                            modifiers: Modifiers::NONE,
                        });
                    }
                    input.events.push(InputEvent::WindowFocused(false));
                }
                Event::MotionNotify(e) => {
                    pointer_position = Some(Pos2::new(e.event_x as f32, e.event_y as f32));
                    let pointer = pointer_position.unwrap() / pixels_per_point;
                    input.events.push(InputEvent::PointerMoved(pointer));
                }
                Event::LeaveNotify(_) => {
                    pointer_position = None;
                    input.events.push(InputEvent::PointerGone);
                }
                Event::ButtonPress(e) | Event::ButtonRelease(e) => {
                    let down = e.response_type & 0x7f == BUTTON_PRESS_EVENT;
                    pointer_position = Some(Pos2::new(e.event_x as f32, e.event_y as f32));
                    let pointer = pointer_position.unwrap() / pixels_per_point;
                    let mods = modifiers(e.state);
                    input.modifiers = mods;
                    if (4..=7).contains(&e.detail) && down {
                        let delta = match e.detail {
                            4 => Vec2::new(0.0, 50.0),
                            5 => Vec2::new(0.0, -50.0),
                            6 => Vec2::new(50.0, 0.0),
                            _ => Vec2::new(-50.0, 0.0),
                        };
                        input.events.push(InputEvent::MouseWheel {
                            unit: egui::MouseWheelUnit::Point,
                            delta,
                            modifiers: mods,
                            phase: egui::TouchPhase::Move,
                        });
                    } else if let Some(button) = match e.detail {
                        1 => Some(PointerButton::Primary),
                        2 => Some(PointerButton::Middle),
                        3 => Some(PointerButton::Secondary),
                        _ => None,
                    } {
                        input.events.push(InputEvent::PointerButton {
                            pos: pointer,
                            button,
                            pressed: down,
                            modifiers: mods,
                        });
                    }
                }
                Event::KeyPress(e) | Event::KeyRelease(e) => {
                    let down = e.response_type & 0x7f == KEY_PRESS_EVENT;
                    let mut mods = modifiers(e.state);
                    let offset = (e.detail.saturating_sub(setup.min_keycode) as usize)
                        * mapping.keysyms_per_keycode as usize;
                    let base = *mapping.keysyms.get(offset).unwrap_or(&0);
                    // X11 reports modifier state before this event. Update Shift
                    // immediately so it also changes speed during a held mouse jog.
                    match base {
                        0xffe1 | 0xffe2 => mods.shift = down,
                        0xffe3 | 0xffe4 => {
                            mods.ctrl = down;
                            mods.command = down;
                        }
                        0xffe9 | 0xffea => mods.alt = down,
                        _ => {}
                    }
                    input.modifiers = mods;
                    let shifted = *mapping.keysyms.get(offset + 1).unwrap_or(&0);
                    let upper = mods.shift ^ e.state.contains(KeyButMask::LOCK);
                    let symbol = if upper && shifted != 0 { shifted } else { base };
                    if let Some(key) = key(base) {
                        let repeat = if down {
                            // Traditional X11 autorepeat sends a release/press
                            // pair with the same timestamp. Detect that pair as
                            // well as servers that emit repeated presses alone.
                            let paired_repeat = last_key_release == Some((e.detail, e.time));
                            last_key_release = None;
                            !pressed.insert(key) || paired_repeat
                        } else {
                            last_key_release = Some((e.detail, e.time));
                            pressed.remove(&key);
                            false
                        };
                        if down && mods.ctrl && key == Key::V {
                            conn.convert_selection(
                                window,
                                clipboard,
                                utf8,
                                paste_property,
                                x11rb::CURRENT_TIME,
                            )?;
                        } else if down && mods.ctrl && key == Key::C {
                            input.events.push(InputEvent::Copy);
                        } else if down && mods.ctrl && key == Key::X {
                            input.events.push(InputEvent::Cut);
                        } else if !(down
                            && repeat
                            && matches!(key, Key::R | Key::P | Key::S)
                            && !mods.ctrl
                            && !mods.alt)
                        {
                            // egui recalculates repeat from its key state, so a
                            // synthetic release/press pair looks like a new press.
                            // Omit repeated transport key events before that step.
                            // Text events below still repeat normally in inputs.
                            input.events.push(InputEvent::Key {
                                key,
                                physical_key: Some(key),
                                pressed: down,
                                repeat,
                                modifiers: mods,
                            });
                        }
                    }
                    if down && !mods.ctrl && !mods.alt && (0x20..0x7f).contains(&symbol) {
                        input.events.push(InputEvent::Text(
                            char::from_u32(symbol).unwrap().to_string(),
                        ));
                    }
                }
                Event::SelectionNotify(e) if e.property != 0 => {
                    let reply = conn
                        .get_property(true, window, paste_property, AtomEnum::ANY, 0, 1024 * 1024)?
                        .reply()?;
                    if let Ok(text) = String::from_utf8(reply.value) {
                        input.events.push(InputEvent::Paste(text));
                    }
                }
                Event::SelectionRequest(e) => {
                    let property = if e.property == 0 {
                        e.target
                    } else {
                        e.property
                    };
                    let accepted = if e.target == targets {
                        conn.change_property32(
                            PropMode::REPLACE,
                            e.requestor,
                            property,
                            AtomEnum::ATOM,
                            &[targets, utf8, AtomEnum::STRING.into()],
                        )?;
                        true
                    } else if e.target == utf8 || e.target == u32::from(AtomEnum::STRING) {
                        conn.change_property8(
                            PropMode::REPLACE,
                            e.requestor,
                            property,
                            e.target,
                            clipboard_text.as_bytes(),
                        )?;
                        true
                    } else {
                        false
                    };
                    conn.send_event(
                        false,
                        e.requestor,
                        EventMask::NO_EVENT,
                        SelectionNotifyEvent {
                            response_type: SELECTION_NOTIFY_EVENT,
                            sequence: 0,
                            time: e.time,
                            requestor: e.requestor,
                            selection: e.selection,
                            target: e.target,
                            property: if accepted { property } else { 0 },
                        },
                    )?;
                }
                _ => {}
            }
        }
        input.focused = focused && !quit;
        current_modifiers = input.modifiers;
        // Zoom can change while the physical mouse is stationary.
        if let Some(pointer) = pointer_position {
            input
                .events
                .push(InputEvent::PointerMoved(pointer / pixels_per_point));
        }
        input
            .viewports
            .entry(egui::ViewportId::ROOT)
            .or_default()
            .native_pixels_per_point = Some(1.0);
        input.screen_rect = Some(Rect::from_min_size(
            Pos2::ZERO,
            Vec2::new(width as f32, height as f32) / pixels_per_point,
        ));
        let output = ctx.run_ui(input, |ui| draw(ui));
        for command in output.platform_output.commands {
            if let egui::OutputCommand::CopyText(text) = command {
                clipboard_text = text;
                conn.set_selection_owner(window, clipboard, x11rb::CURRENT_TIME)?;
            }
        }
        pixels.resize(width as usize * height as usize, [18, 24, 32, 255]);
        let primitives = ctx.tessellate(output.shapes, output.pixels_per_point);
        renderer.render(
            &mut BufferMutRef::new(&mut pixels, width as usize, height as usize),
            &primitives,
            &output.textures_delta,
            output.pixels_per_point,
        );
        let bytes: Vec<u8> = pixels.iter().flatten().copied().collect();
        let stride = width as usize * 4;
        let rows = ((conn.maximum_request_bytes().min(256 * 1024) - 32) / stride).max(1);
        for y in (0..height as usize).step_by(rows) {
            let n = rows.min(height as usize - y);
            conn.put_image(
                ImageFormat::Z_PIXMAP,
                window,
                gc,
                width,
                n as u16,
                0,
                y as i16,
                0,
                screen.root_depth,
                &bytes[y * stride..(y + n) * stride],
            )?;
        }
        conn.flush()?;
        thread::sleep(Duration::from_millis(33).saturating_sub(frame_start.elapsed()));
    }
    Ok(())
}

fn window_icon(size: u32) -> Vec<u32> {
    let mut out = vec![size, size];
    let bolt = [
        (36.0, 8.0),
        (17.0, 35.0),
        (30.0, 35.0),
        (26.0, 56.0),
        (47.0, 26.0),
        (34.0, 26.0),
    ];
    for y in 0..size {
        for x in 0..size {
            let (x, y) = (
                (x as f32 + 0.5) * 64.0 / size as f32,
                (y as f32 + 0.5) * 64.0 / size as f32,
            );
            let mut inside = false;
            let mut previous = bolt[bolt.len() - 1];
            for &(a, b) in &bolt {
                if (b > y) != (previous.1 > y)
                    && x < (previous.0 - a) * (y - b) / (previous.1 - b) + a
                {
                    inside = !inside;
                }
                previous = (a, b);
            }
            let r = (x - 32.0).hypot(y - 32.0);
            let wave = x > 42.0 && y < 28.0 && ((r - 24.0).abs() < 2.0 || (r - 15.0).abs() < 2.0);
            let corner = (x.clamp(16.0, 48.0) - x).hypot(y.clamp(16.0, 48.0) - y);
            out.push(if inside {
                0xffffb95a
            } else if wave {
                0xff4bd3df
            } else if corner < 14.0 {
                0xff141a23
            } else {
                0
            });
        }
    }
    out
}

fn modifiers(mask: KeyButMask) -> Modifiers {
    Modifiers {
        alt: mask.contains(KeyButMask::MOD1),
        ctrl: mask.contains(KeyButMask::CONTROL),
        shift: mask.contains(KeyButMask::SHIFT),
        mac_cmd: false,
        command: mask.contains(KeyButMask::CONTROL),
    }
}

fn key(symbol: u32) -> Option<Key> {
    Some(match symbol {
        0xff08 => Key::Backspace,
        0xff09 => Key::Tab,
        0xff0d | 0xff8d => Key::Enter,
        0xff1b => Key::Escape,
        0xffff => Key::Delete,
        0xff50 => Key::Home,
        0xff57 => Key::End,
        0xff55 => Key::PageUp,
        0xff56 => Key::PageDown,
        0xff51 => Key::ArrowLeft,
        0xff52 => Key::ArrowUp,
        0xff53 => Key::ArrowRight,
        0xff54 => Key::ArrowDown,
        0x20 => Key::Space,
        _ => {
            return match char::from_u32(symbol)?.to_ascii_uppercase() {
                'A' => Some(Key::A),
                'C' => Some(Key::C),
                'F' => Some(Key::F),
                'G' => Some(Key::G),
                'I' => Some(Key::I),
                'M' => Some(Key::M),
                'O' => Some(Key::O),
                'P' => Some(Key::P),
                'R' => Some(Key::R),
                'S' => Some(Key::S),
                'V' => Some(Key::V),
                'X' => Some(Key::X),
                'Y' => Some(Key::Y),
                'Z' => Some(Key::Z),
                _ => None,
            };
        }
    })
}
