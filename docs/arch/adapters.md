# Adapters

As I understood from a description in source files, adapters are needed to organize input/output.
Here description from adapter_manager.h

```
/// Macro to prepare all the necessary adapter functions when adding a
/// new input/output. For example when you want to listen to
/// car_status message for your module, you can do
/// REGISTER_ADAPTER(CarStatus) write an adapter class called
/// CarStatusAdapter, and call EnableCarStatus(`car_status_topic`,
/// true, `callback`(if there's one)) in AdapterManager.

```

Let see by example of adding new listeners of CompressedImage topic into perception module.

First of all, there is CompressedImage adapter already.

In file `/apollo/modules/common/adapters/message_adapters.h` there is the following line:
```
using CompressedImageAdapter = Adapter<sensor_msgs::CompressedImage>;
```
In file `/apollo/modules/common/adapters/adapter_manager.h` we have to register the adapter:
```
REGISTER_ADAPTER(CompressedImage);
``` 
In file  `/apollo/modules/common/adapters/adapter_gflags.h` we declare a flag to keep topic name:
```
DECLARE_string(compressed_image_topic);
```
In file  `/apollo/modules/common/adapters/adapter_gflags.cc` we define a flag to keep topic name:
```
DEFINE_string(compressed_image_topic, "camera/image_raw",
              "CompressedImage topic name");
```

In file `/home/alex/workspace/apollo/modules/common/adapters/adapter_manager.cc` in function `void AdapterManager::Init` there is a initialization all adapters for module.
Here is example for CompressedImage adapter:
```
case AdapterConfig::COMPRESSED_IMAGE:
    EnableCompressedImage(FLAGS_compressed_image_topic, config);
    break;
```

A file with configuration of all adapters needed for certain module is `adapter.conf` in `conf` folder of corresponding module.
For instance, to set up nesesary adapters for preception module you have to edit `/apollo/modules/perception/conf/adapter.conf` file.
I have added there the following lines to add CompressedImage adapter:
```
config {
  type: COMPRESSED_IMAGE
  mode: RECEIVE_ONLY
  message_history_limit: 1
}
```

To add a callback for an adapter you have to call the following line:
```
AdapterManager::AddCompressedImageCallback(&CameraProcessSubnode::ImgCompressCallback, this); 
```

Also you can change default value for adapter topic using corresponding conf file for needed module.
For instance, I have added the folowing line:
```
--compressed_image_topic=/apollo/sensor/camera/obstacle/front_6mm/compressed 
```
into `/apollo/modules/perception/conf/perception.conf` file