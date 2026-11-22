The packaged descriptors contain the upstream referee, camera, tracker and AutoRef CI messages from
[ssl-game-controller v3.23.1](https://github.com/RoboCup-SSL/ssl-game-controller/tree/v3.23.1/proto).
The accompanying GPL-3.0 license applies to these upstream definitions.
To regenerate from that tag, run the following with its proto directory as the include path.

```sh
protoc -I /path/to/ssl-game-controller/proto --include_imports \
  --descriptor_set_out=referee.pb state/ssl_gc_referee_message.proto
protoc -I /path/to/ssl-game-controller/proto --include_imports \
  --descriptor_set_out=vision.pb ci/autoref/ssl_autoref_ci.proto vision/ssl_vision_wrapper.proto
```
