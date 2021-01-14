# Sample code to test PCL and Velodyne Point Cloud sensor

Initially install `velodyne_simulator` package

```
sudo apt install ros-melodic-velodyne-simulator
```

## Considerations

Normally the approaches involve segmenting the floor and filtering that out, so the core features are still present. After that another algorithm must be used to properly retain the desired pattern/objective

It is a good practice to apply some filter before, like the Voxel Grid to make a downsample


## Useful tools

Libraries:
- pcl-tools