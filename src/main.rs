use cnc_brain::serialport::{self, Serialport};
use grbl_compat::{gcode_parser, grbl_state};

fn main() -> Result<(), anyhow::Error> {
    let mut port = serialport::MockSerialport::default();
    port.write("G0 X10 F30\n".as_bytes());
    port.write("G1 X20 Y15 F60\n".as_bytes());
    port.write("G1 X50 Y25 F60\n".as_bytes());
    port.write("G10 L2 P0 X99 Y15 F60\n".as_bytes());
    port.write("G55 Y99 X99\n".as_bytes());
    port.write("G0 G1 X99\n".as_bytes());

    run_machine(port)?;

    Ok(())
}

fn run_machine(mut port: serialport::MockSerialport) -> Result<(), anyhow::Error> {
    let mut parser = gcode_parser::GCodeParser::default();

    let buffer = port.read();
    let lines = String::from_utf8_lossy(&buffer);

    let mut machine = grbl_state::GrblState::default();

    for line in lines.lines() {
        match parser.parse_line(line) {
            Ok(words) => {
                if let Ok(command) = parser.build_command(words) {
                    machine.apply_state(command);
                } else {
                    println!("Failed to execute '{}'", line);
                }
            }
            Err(err) => {
                println!("Errors in line '{}': {}", line, err);
            }
        }
    }

    println!("End state: {:?}", machine);

    Ok(())
}
