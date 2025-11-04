{
  description = "Daisy Embassy - async audio development with daisy seed and embassy";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, rust-overlay }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };

        # Use the specific Rust version from rust-toolchain
        rustToolchain = pkgs.rust-bin.stable."1.85.0".default.override {
          extensions = [ "rust-src" "clippy" "rustfmt" "llvm-tools-preview" ];
          targets = [ "thumbv7em-none-eabihf" ];
        };

        # Build inputs for the development environment
        buildInputs = with pkgs; [
          rustToolchain
          cargo-nextest
          cargo-expand
          cargo-watch
          probe-rs-tools
          dfu-util
          openocd
          gdb
          pkg-config
          udev
          systemd

          # MIDI tools
          alsa-utils      # aconnect, aplaymidi, amidi
          qjackctl        # JACK MIDI routing
          vmpk            # Virtual MIDI Piano Keyboard
          fluidsynth      # Software synthesizer for testing
          soundfont-fluid # Default soundfont
        ];

        nativeBuildInputs = with pkgs; [
          pkg-config
        ];

      in
      {
        devShells.default = pkgs.mkShell {
          inherit buildInputs nativeBuildInputs;

          # Environment variables
          RUST_SRC_PATH = "${rustToolchain}/lib/rustlib/src/rust/library";
          DEFMT_LOG = "trace";

          # udev rules for probe-rs and debugging hardware
          UDEV_RULES_PATH = "${pkgs.probe-rs-tools}/lib/udev/rules.d";

          shellHook = ''
            echo "🦀 Daisy Embassy development environment"
            echo "Rust toolchain: ${rustToolchain.name}"
            echo "Target: thumbv7em-none-eabihf (Cortex-M7F)"
            echo ""
            echo "Available commands:"
            echo "  nix develop -c cargo build --release --example NAME              # Build example"
            echo "  nix develop -c cargo nextest run                                 # Run tests"
            echo "  nix develop -c dfu-util -l                                       # List DFU devices"
            echo "  nix develop -c probe-rs list                                     # List connected probes"
            echo ""
            echo "Daisy Seed flashing workflow:"
            echo "  1. cargo build --release --example NAME"
            echo "  2. rust-objcopy -O binary target/thumbv7em-none-eabihf/release/examples/NAME target/thumbv7em-none-eabihf/release/examples/NAME.bin"
            echo "  3. dfu-util -a 0 -s 0x08000000 -D target/thumbv7em-none-eabihf/release/examples/NAME.bin"
            echo ""
            echo "For DFU mode: Hold BOOT + RESET, release RESET, then release BOOT"
            echo "For normal mode: Press RESET button"
            echo ""
            echo "Make sure your user is in the 'plugdev' group for USB access:"
            echo "  sudo usermod -a -G plugdev $USER"
            echo ""
          '';
        };

        # Optional: Package the project itself
        packages.default = pkgs.rustPlatform.buildRustPackage {
          pname = "daisy-embassy";
          version = "0.2.1";
          src = ./.;
          cargoLock.lockFile = ./Cargo.lock;

          nativeBuildInputs = nativeBuildInputs;
          buildInputs = buildInputs;

          # This is a no_std embedded library, so we don't need to build binaries
          doCheck = false;
          buildPhase = ''
            cargo check --target thumbv7em-none-eabihf
          '';

          installPhase = ''
            mkdir -p $out
            echo "This is an embedded library package - no binaries to install" > $out/README
          '';
        };

        # Formatting
        formatter = pkgs.nixfmt-rfc-style;
      });
}