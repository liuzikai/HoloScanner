HoloScanner
===========

```shell
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make DepthToPCD
```

## Known Issues

* Depth filtering based on hand position does not work when the object is getting too close or too far from the
  HoloLens, although the depth clipping range seems right.
* While doing the scanning, very fast movement of the object causes the registration to lose tracking. An orange point
  cloud is sent to the HoloLens, but then the TCP connection seems corrupted and cannot be automatically recovered with
  the state machine in UnitySample/Assets/Scipts/TCPClient.cs. This seems not happening when the point cloud normally
  loses tracking, but only with intentional fast movement by the user.
* Eye tracking data streaming from HL2UnityPlugin/HL2Interactions to TCPDataSource is not tested yet.
* FileSystemSource is out of maintenance.

## Using the 'score-denoise' denoiser

The denoising repository requires some python packages.
The easiest way to get them is to just run the following:

```
cd CPP/ext/score-denoise
conda env create -f env.yml
conda activate score-denoise
cd -
```