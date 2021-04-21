############ QDSP6V2 ############

obj-$(CONFIG_SND_SOC_MSM_QDSP6V2_INTF) += audio_q6.o
audio_q6-y := adsp_err.o audio_calibration.o audio_cal_utils.o		\
	      audio_slimslave.o avtimer.o msm_audio_ion.o q6adm.o	\
	      q6afe.o q6asm.o q6audio-v2.o q6core.o q6_init.o q6lsm.o	\
	      q6voice.o rtac.o
audio_q6-$(CONFIG_DTS_SRS_TM) += msm-dts-srs-tm-config.o

obj-$(CONFIG_MSM_ADSP_LOADER) += audio_adsp_loader.o
audio_adsp_loader-y := adsp-loader.o

obj-$(CONFIG_MSM_QDSP6_NOTIFIER) += audio_q6_notifier.o
audio_q6_notifier-y := audio_notifier.o

obj-$(CONFIG_MSM_QDSP6_PDR) += audio_q6_ssr.o
audio_q6_ssr-y := audio_ssr.o

obj-$(CONFIG_MSM_QDSP6_PDR) += audio_q6_pdr.o
audio_q6_pdr-y := audio_pdr.o

obj-$(CONFIG_MSM_ULTRASOUND) += audio_usf.o
audio_usf-y := usf.o usfcdev.o q6usm.o

obj-y += codecs/

EXTRA_CFLAGS += -I$(srctree)/techpack/audio/include/uapi	\
		-I$(srctree)/techpack/audio/include
