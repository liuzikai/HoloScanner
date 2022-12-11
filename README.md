# HoloScanner

```shell
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make DepthToPCD
```

* Hand strictly tracked is put as early as possible

## Using the 'score-denoise' denoiser

The denoising repository requires some python packages.
The easiest way to get them is to just run the following:

```
cd CPP/ext/score-denoise
conda env create -f env.yml
conda activate score-denoise
cd -
```