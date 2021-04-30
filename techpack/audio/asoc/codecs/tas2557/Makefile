obj-$(CONFIG_SND_SOC_TAS2557) += snd-soc-tas2557.o
snd-soc-tas2557-y := tas2557-core.o tiload.o
snd-soc-tas2557-$(CONFIG_TAS2557_CODEC) += tas2557-codec.o
snd-soc-tas2557-$(CONFIG_TAS2557_MISC) += tas2557-misc.o
snd-soc-tas2557-$(CONFIG_TAS2557_REGMAP) += tas2557-regmap.o

ccflags-y += -Itechpack/audio/asoc/codecs
