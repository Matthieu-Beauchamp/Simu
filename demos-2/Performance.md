<!--TODO: add git tags for the tested revisions-->

## Basic performance tracking for demos

All data was collected from a Windows 11 PC with an
Intel(R) Core(TM) i7-10700F CPU @ 2.90GHz (2.90 GHz)
CPU and 16GB RAM.

The build uses CMake `Release` configuration with no additional flags.

The given times and memory usages include resources used for rendering the demo.
Frame time does not go lower than 6-7 ms because of Vsync.
Frame times of 6 indicate a well-supported load.

### Old engine (`aa4b1b1a3b56e487ffaa5647bde433b5133ffd17`)

Frame times are taken from the timer shown in the demos.
Memory usage is taken from the Windows task manager.

#### Box Stack

| parameters (stacks x height) | frame time (ms) | memory usage (MB) |
|------------------------------|-----------------|-------------------|
| 100 x 100                    | 52              | 60                |
| 75 x 75                      | 26              | 54                |
| 50 x 50                      | 11              | 48                |
| 40 x 40                      | 6               | 45                |
| 25 x 25                      | 6               | 42                |
| 10 x 10                      | 6               | 41                |

### New Engine (`65c2223023f0e449eee797a379071d39677ad6a4`)

Frame times are taken from the timer shown in the demos.
Memory usage is taken from the Windows task manager.

#### Box Stack

| parameters (stacks x height) | frame time (ms) | memory usage (MB) |
|------------------------------|-----------------|-------------------|
| 100 x 100                    | 52              | 50                |
| 75 x 75                      | 30              | 49                |
| 50 x 50                      | 12              | 45                |
| 40 x 40                      | 8               | 44                |
| 25 x 25                      | 6               | 42                |
| 10 x 10                      | 6               | 40                |

