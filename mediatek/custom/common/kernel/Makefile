include $(MTK_PATH_BUILD)/common.mk
$(call all-modules-src-or-makefile,$(obj),CUSTOM_KERNEL_)
obj-n := dummy.o
$(obj)/ft_gesture_lib.a.o: $(srctree)/../$(obj)/touchpanel/ft_gesture_lib.a
	cp $(srctree)/../$(obj)/touchpanel/ft_gesture_lib.a $(obj)/ft_gesture_lib.a.o
