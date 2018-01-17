# Scripts
---

Useful scripts for automating things.

### Copy binary's shared objects to [libs](libs) directory to be loaded at runtime:
```bash
./copy_libs.sh
```

The [libs](libs) directory is added to `LD_LIBRARY_PATH` in start scripts so that shared objects within the directory will be loaded at runtime.  Adding shared objects to this directory ensures that they will be found when copying to and running the binary on another computer such as for a competition.  Note that some libraries may not be compatible with later/future OS releases (e.g. the default version of libc on Ubuntu 14.04 is not compatible with Ubuntu 16.04).

### Copy files needed to run binary to a new directory:
```bash
./copy_files_needed_to_run.sh <destination>
```

Copies executable, libraries, scripts, and parameter and skill files needed to run the binary to the `destination` directory.  Useful for packaging the binary to be run on another computer such as for a competition.
