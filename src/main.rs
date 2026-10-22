mod app;
mod controller;
mod desktop;
mod file_dialog;
mod gcode;
mod protocol;
mod simulator;
fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--help" || a == "-h") {
        println!(
            "Plasma WiFi\nUsage: plasma-wifi [--demo] [--controller HOST:PORT] [--http URL] [FILE]\n\n--demo connects only to an in-process simulator and loads an example job.\nThe normal default is 192.168.0.1:23, HTTP port 80."
        );
        return Ok(());
    }
    let demo = args.iter().any(|a| a == "--demo");
    let mut settings = if demo {
        app::Settings::default()
    } else {
        app::Settings::load()
    };
    let _sim = if demo {
        let sim = simulator::Simulator::start()?;
        settings.address = sim.address.clone();
        settings.http = sim.http.clone();
        eprintln!("Simulator: TCP {} HTTP {}", sim.address, sim.http);
        Some(sim)
    } else {
        None
    };
    let mut file = None;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--demo" => {}
            "--controller" | "--http" => {
                anyhow::ensure!(
                    !demo,
                    "--demo cannot be combined with controller address overrides"
                );
                let flag = &args[i];
                i += 1;
                let value = args
                    .get(i)
                    .ok_or_else(|| anyhow::anyhow!("{flag} needs a value"))?
                    .clone();
                if flag == "--controller" {
                    settings.address = value;
                } else {
                    settings.http = value;
                }
            }
            arg if arg.starts_with('-') => anyhow::bail!("Unknown option {arg}"),
            _ => file = Some(std::path::PathBuf::from(&args[i])),
        }
        i += 1;
    }
    let mut app = app::App::new(settings, demo);
    if let Some(file) = file {
        app.open(file);
    } else if demo {
        app.load_text("rounded-plate.nc", simulator::DEMO);
    }
    desktop::run(|ui| app.ui(ui))
}
