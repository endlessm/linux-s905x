#! /bin/bash

export CROSS_COMPILE=/opt/gcc-linaro-6.3.1-2017.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-

make ARCH=arm64 gxm_q200_2g.dtb || echo "Compile dtb Fail !!"

make ARCH=arm64 gxm_skt.dtb || echo "Compile dtb Fail !!"

make ARCH=arm64 gxl_p212_2g.dtb || echo "Compile dtb Fail !!"

make ARCH=arm64 gxl_p212_1g.dtb || echo "Compile dtb Fail !!"

make ARCH=arm64 gxl_p400_2g.dtb || echo "Compile dtb Fail !!"

make ARCH=arm64 gxl_p401_2g.dtb || echo "Compile dtb Fail !!"

make ARCH=arm64 gxl_skt.dtb || echo "Compile dtb Fail !!"

make ARCH=arm64 axg_pxp.dtb || echo "Compile dtb Fail !!"
