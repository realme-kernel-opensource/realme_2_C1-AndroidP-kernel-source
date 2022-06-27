ccflags-y += -I$(src)			# needed for trace events

obj-y					+= ion/

obj-$(CONFIG_ASHMEM)			+= ashmem.o

#ifdef VENDOR_EDIT
obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= oppo_lowmemorymonitor.o
#endif

obj-$(CONFIG_ANDROID_LOW_MEMORY_KILLER)	+= lowmemorykiller.o
obj-$(CONFIG_ANDROID_VSOC)		+= vsoc.o
