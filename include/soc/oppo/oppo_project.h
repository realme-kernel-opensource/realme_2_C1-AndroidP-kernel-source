#ifdef WT_COMPILE_FACTORY_VERSION
/* 
 *
 * yixue.ge add for oppo project
 *
 *
 */
#ifndef _OPPO_PROJECT_H_
#define _OPPO_PROJECT_H_

typedef enum OPPO_PROJECT OPPO_PROJECT;

typedef struct
{
  unsigned int                  nProject;
  unsigned char                 nModem;
  unsigned char                 nOperator;
  unsigned char                 nPCBVersion;
  unsigned char                 nBootMode;
} ProjectInfoCDTType;

#ifdef CONFIG_OPPO_COMMON_SOFT
unsigned int init_project_version(void);
unsigned int get_project_from_smem(void);
#else
unsigned int init_project_version(void) { return 0;}
unsigned int get_project_from_smem(void) { return 0;}
#endif
#endif

#endif
