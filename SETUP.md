# Setup Rust

`curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`

`rustup target add thumbv6m-none-eabi && sudo apt update && sudo apt upgrade -y && sudo apt install -y libudev-dev lld && cargo install elf2uf2-rs --locked`
