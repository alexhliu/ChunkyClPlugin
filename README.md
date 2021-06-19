[![Stable](https://github.com/alexhliu/ChunkyClPlugin/actions/workflows/stable.yml/badge.svg)](https://github.com/alexhliu/ChunkyClPlugin/actions/workflows/stable.yml)
[![Development](https://github.com/alexhliu/ChunkyClPlugin/actions/workflows/development.yml/badge.svg)](https://github.com/alexhliu/ChunkyClPlugin/actions/workflows/development.yml)


# ChunkyCL

ChunkyCL is a plugin to the Minecraft Path-tracer [Chunky](https://github.com/chunky-dev/chunky) which harnesses the power of the GPU with OpenCL 1.2+ to accelerate rendering.
It is currently a work in progress and does not support many features such as non-cube blocks and biome tinting. The core renderer itself is still under development
so render results may change drastically between versions.

## Downloads
* [Latest mostly stable build](https://nightly.link/alexhliu/ChunkyClPlugin/workflows/stable/stable/ChunkyClPlugin.zip)
* [Latest development build](https://nightly.link/alexhliu/ChunkyClPlugin/workflows/development/master/ChunkyClPlugin.zip)

<sub><sup>Note: Even if the build is failing, the link will link to the latest successful build. This will work with some older snapshots of Chunky.</sup></sub>

## Installation

### Note: The latest version requires at least snapshot `2.4.0-157-g30c7e879`
Download the latest plugin build and extract it. In the Chunky Launcher, expand `Advanced Settings` and click on `Manage plugins`. In the `Plugin Manager` window click on `Add` and select the `.jar` file in the extracted zip file. Click on `Save` and start Chunky as usual.

![image](https://user-images.githubusercontent.com/42661490/116319916-28ef2580-a76c-11eb-9f93-86d444a349fd.png)

To download the latest Chunky snapshot, open `Advanced Settings` in the Chunky Launcher and enable `Download Snapshots`. Then click on the `Check for update` button.

![image](https://user-images.githubusercontent.com/42661490/116319834-0e1cb100-a76c-11eb-8842-e72dc2b76854.png)

Select `ChunkyCL` as your renderer for the scene in the `Advanced` tab.

![image](https://user-images.githubusercontent.com/42661490/122492084-fc040580-cf99-11eb-9b08-b166dc25db41.png)

### Performance

Rough performance with a RTX 2070 is around 10 times that of the traditional CPU renderer as of 2021-02-02.

Decreasing `Render threads` or `CPU utilization` may improve GPU performance. A good starting point is 1 `render thread` at 100% `CPU utilization`.

## Compatibility

* Not compatible with the Denoising Plugin.
