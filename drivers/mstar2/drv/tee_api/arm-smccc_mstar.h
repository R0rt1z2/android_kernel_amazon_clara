#ifndef __LINUX_ARM_SMCCC_MSTAR_H
#define __LINUX_ARM_SMCCC_MSTAR_H

#include <linux/linkage.h>
#include <linux/types.h>

/**
 * struct arm_smccc_res_mstar - Result from SMC/HVC call
 * @a0-a3 result values from registers 0 to 3
 */
#if 0
struct arm_smccc_res_mstar {
	unsigned long a0;
	unsigned long a1;
	unsigned long a2;
	unsigned long a3;
};
#endif


/**
 * arm_smccc_smc_mstar() - make SMC calls
 * @a0-a7: arguments passed in registers 0 to 7
 * @res: result values from registers 0 to 3
 *
 * This function is used to make SMC calls following SMC Calling Convention.
 * The content of the supplied param are copied to registers 0 to 7 prior
 * to the SMC instruction. The return values are updated with the content
 * from register 0 to 3 on return from the SMC instruction.
 */
asmlinkage void arm_smccc_smc_mstar(unsigned long a0, unsigned long a1,
			unsigned long a2, unsigned long a3, unsigned long a4,
			unsigned long a5, unsigned long a6, unsigned long a7,
			struct arm_smccc_res_mstar *res);

#endif /*__LINUX_ARM_SMCCC_MSTAR_H*/
