# Native standalone ELF startup

`crt1` is the application entry object for RV32, RV64 and AArch64. It reads
`argc` and `argv` from the native ELF initial stack and calls:

```text
__libc_start_main(main, argc, argv, NULL, NULL, rtld_fini, initial_sp)
```

The RISC-V entry establishes the application's `gp`. Both entries preserve the
optional `rtld_fini` register, keep the initial stack aligned and trap if the
non-returning libc entry unexpectedly returns. Relocation is already complete
before `_start`; the CRT does not load dependencies or relocate itself.

## Build an application

```gn
import("//build/templates/static_pie_app.gni")

static_pie_app("my_app") {
  sources = [ "src/main.rs" ]
}
```

The Rust root links librs explicitly, then uses an ordinary Rust entry:

```rust
extern crate librs;

fn main() {
    let worker = std::thread::spawn(|| vec![42; 32]);
    assert_eq!(worker.join().unwrap().len(), 32);
}
```

The template supplies `crt1` and `librs_swi`, PIC code, static PIE linking and
an explicit `_start` entry. The CRT archive is retained through `-u _start`.
It uses the board's application linker configuration, without its kernel
linker script or `rsrt`. Additional dependencies must also support PIC.
`std` supplies Rust allocation and panic support; a `no_std` root supplies its
own allocator and panic handler. The existing firmware `rsrt` and
`__librs_start_main()` path remains separate.

## Load and finish

The kernel loads the file through `loader::load_image`, constructs
`StartupStorage` and converts it using unsafe `into_application()`. That
operation requires trusted native code and all image users to remain within
the application lifetime. `Application::start(None)` starts a direct RTOS
application; `Some(existing_process)` associates its threads with an existing
Process and its file descriptors.

Libc initializes the image, calls Rust's generated C `main` wrapper, waits for
cooperative child cleanup and executes `atexit`/finalizers. The application
owner releases the image after all member cleanup. `Application::wait()` then
reports normal status or aborted execution. Child threads must join or detach
and be able to finish without the main thread running.

`std_app` exercises this production CRT with a normal Rust `fn main`, environment
access, heap growth/zeroing, a std thread and its TLS destructor, finalization
and a nonzero `ExitCode`. `runtime_app` retains an independent test entry to
exercise libc's optional legacy init/fini arguments. The raw `startup_app`
checks the initial stack independently of libc.

The current BlueOS Rust std sources select an empty `std::env::args()` iterator.
The CRT passes native arguments to libc and Rust's generated wrapper, and the
fixture checks them in its constructor; exposing them through `std::env::args()`
requires a separate toolchain update. `std::env::var()` works through librs.
This standalone path uses BlueOS syscalls and its libc implementation; it is
not an assertion of Linux/glibc binary compatibility.
