#
# This file is the zynqv2xfinal recipe.
#

SUMMARY = "Simple zynqv2xfinal application"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://Makefile \
           file://zynqv2xfinal.c\
           \
           file://mmap_xil_io.h \
           file://xil_assert.h \
           file://xil_types.h \
           file://xstatus.h \
           \
           file://PmodGPS.c \
           file://PmodGPS.h \
           file://xuartns550.c \
           file://xuartns550.h \
           file://xuartns550_l.c \
           file://xuartns550_l.h \
           file://xuartns550_i.h \
           file://xuartns550_stats.c \
           file://xuartns550_selftest.c \
           file://xuartns550_options.c \
           \
           file://PmodNAV.c \
           file://PmodNAV.h \
           file://xspi.c \
           file://xspi.h \
           file://xspi_l.h \
           file://xspi_i.h \
           file://xspi_options.c \
           \
           file://PmodOLED.c \
           file://PmodOLED.h \
           file://OledGrph.c \
           file://OledChar.c \
           file://OledDriver.c \
           file://FillPat.c \
           file://ChrFont0.c \
          "

S = "${WORKDIR}"

do_compile() {
	     oe_runmake
}

do_install() {
	     install -d ${D}${bindir}
	     install -m 0755 zynqv2xfinal ${D}${bindir}
}
