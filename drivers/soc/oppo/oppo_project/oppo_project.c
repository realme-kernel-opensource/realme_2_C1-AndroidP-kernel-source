#ifdef WT_COMPILE_FACTORY_VERSION
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <soc/qcom/smem.h>
#include <soc/oppo/oppo_project.h>
#include <linux/io.h>
#include <linux/mm.h>


static ProjectInfoCDTType *format = NULL;

unsigned int init_project_version(void)
{
	unsigned int len = (sizeof(ProjectInfoCDTType) + 3)&(~0x3);

	format = (ProjectInfoCDTType *)smem_alloc(SMEM_PROJECT,len,0,0);

	if(format)
		return format->nProject;

	return 0;
}


unsigned int get_project_from_smem(void)
{
	if(format)
		return format->nProject;
	else
		return init_project_version();
	return 0;
}


MODULE_DESCRIPTION("OPPO project version");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Joshua <gyx@oppo.com>");
#endif
