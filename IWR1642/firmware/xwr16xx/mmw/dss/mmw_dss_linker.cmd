/*----------------------------------------------------------------------------*/
/* Linker Settings                                                            */
--retain="*(.intvecs)"

/*----------------------------------------------------------------------------*/
/* Section Configuration                                                      */
SECTIONS
{
    systemHeap : {} >> L2SRAM_UMAP0 | L2SRAM_UMAP1
    .l3ram: {} >> L3SRAM
    .dpc_l1Heap  : { } > L1DSRAM
    .dpc_l2Heap: { } >> L2SRAM_UMAP0 | L2SRAM_UMAP1
    .demoSharedMem: { } >> HSRAM
}
/*----------------------------------------------------------------------------*/

