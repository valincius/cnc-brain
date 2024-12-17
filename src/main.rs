use cnc_brain::{
    gcode_parser, machine_state,
    serialport::{self, Serialport},
};

fn main() -> Result<(), anyhow::Error> {
    let mut port = serialport::MockSerialport::default();
    port.write("G0 X10 F30\n".as_bytes());
    port.write("G1 X20 Y15 F60\n".as_bytes());
    port.write("G1 X50 Y25 F60\n".as_bytes());
    port.write("G10 L2 P1 Y15 F60\n".as_bytes());
    port.write("G55 Y99 X99\n".as_bytes());
    port.write("G0 G1 X99\n".as_bytes());

    run_machine(port)?;

    Ok(())
}

fn run_machine(mut port: serialport::MockSerialport) -> Result<(), anyhow::Error> {
    let mut parser = gcode_parser::GCodeParser::new();

    let buffer = port.read();
    let lines = String::from_utf8_lossy(&buffer);

    let mut machine = machine_state::MachineState::new();

    for line in lines.lines() {
        match parser.process_line(line) {
            Ok(state) => {
                machine.apply_state(state);
            }
            Err(err) => {
                println!("Errors in line '{}': {}", line, err);
            }
        }
    }

    println!("End state: {:?}", machine);

    Ok(())
}
