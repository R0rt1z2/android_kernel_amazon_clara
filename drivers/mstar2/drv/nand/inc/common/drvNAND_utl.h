#ifndef __DRVNAND_UTL_H__
#define __DRVNAND_UTL_H__

#include "drvNAND.h"

extern U32 drvNAND_ProbeReadSeq(void);
extern U32 drvNAND_CheckPartInfo(U32 u32_PageIdx);
extern U32 drvNAND_SearchCIS(void);
extern U32 drvNAND_InitBBT(INIT_BBT_t *pInitBBT);
extern void drvNAND_ParseNandInfo(NAND_FLASH_INFO_t *pNandInfo);


typedef UNFD_PACK0 struct _TEST_ALIGN_PACK {

	U8	u8_0;
	U16	u16_0;
	U32	u32_0, u32_1;

} UNFD_PACK1 TEST_ALIGN_PACK_t;
extern U32 drvNAND_CheckAlignPack(U8 u8_AlignByteCnt);

extern void dump_mem(unsigned char *buf, int cnt);
extern void dump_nand_info(NAND_FLASH_INFO_t *pNandInfo);
extern void dump_part_records(PARTITION_RECORD_t *pRecord, int cnt);
extern void dump_part_info(PARTITION_INFO_t *pPartInfo);
extern void dump_miu_records(MIU_RECORD_t *records, int cnt);
extern void dump_miu_part(MIU_PART_t *pMiuPart);
extern void dump_general_blk_info(BLK_INFO_t *pBlkInfo);
extern void dump_nand_driver(NAND_DRIVER *pNandDrv);

extern U32 drvNAND_CompareCISTag(U8 *tag);
extern U8  drvNAND_CountBits(U32 u32_x);
extern U32 drvNAND_CheckSum(U8 *pu8_Data, U16 u16_ByteCnt);

extern PARTITION_RECORD_t *drvNAND_SearchPartition(PARTITION_RECORD_t *pRecord,
												   U16 u16_PartType);

extern U32 drvNAND_ReadBlk(U8 *pu8_DestAddr, U8 u8_PartType, U16 u16_PBA,
							U16 u16_LBA, U32 u32_StartSector, 
							U32 u32_SectorCnt);
extern U32 drvNAND_WriteBlk(U8 *pu8_DestAddr, U8 u8_PartType, U16 u16_PBA,
							U16 u16_LBA, U32 u32_StartSector,
							U32 u32_SectorCnt);

extern U32 drvNAND_WriteBlkNonBackup(U8 *pu8_DestAddr, U8 u8_PartType, U16 u16_PBA,
					 U16 u16_LBA,	U32 u32_StartSector,
					 U32 u32_SectorCnt);

extern U32 drvNAND_IsGoodBlk(U16 u16_BlkIdx);
extern U32 drvNAND_IsFreeBlk(U16 u16_BlkIdx);
extern U8  drvNAND_CheckAll0xFF(U8* pu8_Buf, U32 u32_ByteCnt);
extern U16 drvNAND_FindFreePage(U16 u16_BlkIdx);
extern 	U16 drvNAND_FindFreeLSBPageIdx(U16 u16_PBA);
extern U32 drvNAND_MoveBlkData(U16 u16_DestBlk, U16 u16_SrcBlk, U16 u16_PageCnt);

extern U32 drvNAND_MarkBadBlk(U16 u16_BlkIdx);
extern U32 drvNAND_ErasePhyBlk(U16 u16_BlkIdx);
extern U32 drvNAND_ReadPhyPage(U32 u32_PageIdx, U8 *pu8_Data, U8 *pu8_Spare);
extern U32 drvNAND_WritePhyPage(U32 u32_PageIdx, U8 *pu8_Data, U8 *pu8_Spare);
extern int nand_ReadDisturbance_BigImg(U32  u32_BlkRow, U32 u32_BakBlkRow, U32 (*nand_markbad)(U32), U8 u8_CustSpareMarker, U8 u8_CustMarkerIndex);
extern U16 drvNAND_BBT_Rev_StartBlk(void);
extern U16 drvNAND_GetBackupBlk(void);
extern int nand_CheckEmptyPage(U8 *spare);
extern U32 drvNAND_IsPageLP(U16 PIdx, U16 *PairedPageIdx);
extern U32 drvNAND_WriteDummyToPages(U16 StartPIdx, U16 EndPIdx, U32 u32_BlkRow);
extern int nand_CheckEmptySectorsFalseAlarm(U8 *main, U8 *spare, U16 u16_SectorCnt);
extern int nand_CheckEmptyPageFalseAlarm(U8 *main, U8 *spare);

#endif /* __DRVNAND_UTL_H__ */
