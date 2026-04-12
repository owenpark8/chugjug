#include "stm32u5xx_hal.h"
#include "stm32u5xx_hal_dma.h"

/*
(+) DMA linear transfer :
    Linked-list queue building
    (++) HAL_DMAEx_List_BuildNode()
    (++) HAL_DMAEx_List_InsertNode_Tail()
                    .
                    .
                    .
    (++) HAL_DMAEx_List_BuildNode()
    (++) HAL_DMAEx_List_InsertNode_Tail()
    (++) HAL_DMAEx_List_ConvertQToDynamic()
    Linked-list queue execution
    (++) HAL_DMAEx_List_Init()
    (++) HAL_DMAEx_List_LinkQ()
    (++) HAL_DMAEx_List_Start() / HAL_DMAEx_List_Start_IT()
    (++) HAL_DMAEx_List_UnLinkQ()
    (++) HAL_DMAEx_List_DeInit()
*/


template<size_t NumSamples>
class SoundEffect {
    static constexpr size_t m_num_samples = NumSamples;
    static constexpr uint16_t m_samples_per_node = 32736;
    static constexpr size_t m_num_nodes = (NumSamples + m_samples_per_node - 1) / m_samples_per_node;
    DMA_NodeTypeDef m_sai_dma_nodes[m_num_nodes];
    DMA_QListTypeDef m_sai_dma_queue;
    DMA_HandleTypeDef* m_dma_handle;
    SAI_HandleTypeDef* m_sai_handle;
    int16_t const* m_sample_buffer;

public:
    SoundEffect(int16_t const* sample_buffer, DMA_HandleTypeDef* dma_handle, SAI_HandleTypeDef* sai_handle)
        : m_sai_dma_nodes{},
          m_sai_dma_queue{},
          m_dma_handle(dma_handle),
          m_sai_handle(sai_handle),
          m_sample_buffer(const_cast<int16_t*>(sample_buffer)) {}

    bool init() {
        __HAL_LINKDMA(m_sai_handle, hdmatx, *m_dma_handle);

        DMA_NodeConfTypeDef pNodeConfig = {0};
        pNodeConfig.NodeType = DMA_GPDMA_LINEAR_NODE;
        pNodeConfig.Init.Request = GPDMA1_REQUEST_SAI1_A;
        pNodeConfig.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
        pNodeConfig.Init.Direction = DMA_MEMORY_TO_PERIPH;
        pNodeConfig.Init.SrcInc = DMA_SINC_INCREMENTED;
        pNodeConfig.Init.DestInc = DMA_DINC_FIXED;
        pNodeConfig.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_HALFWORD;
        pNodeConfig.Init.DestDataWidth = DMA_DEST_DATAWIDTH_HALFWORD;
        pNodeConfig.Init.SrcBurstLength = 1;
        pNodeConfig.Init.DestBurstLength = 1;
        pNodeConfig.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1 | DMA_DEST_ALLOCATED_PORT0;
        pNodeConfig.Init.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
        pNodeConfig.TriggerConfig.TriggerPolarity = DMA_TRIG_POLARITY_MASKED;
        pNodeConfig.DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
        pNodeConfig.DataHandlingConfig.DataAlignment = DMA_DATA_RIGHTALIGN_ZEROPADDED;

        for (size_t i = 0; i < m_num_nodes; i++) {
            size_t const offset_samples = i * m_samples_per_node;
            size_t const remaining = NumSamples - (i * m_samples_per_node);
            size_t const count = (remaining > m_samples_per_node) ? m_samples_per_node : remaining;

            pNodeConfig.SrcAddress = (uint32_t) (m_sample_buffer + offset_samples);
            pNodeConfig.DstAddress = (uint32_t) (&m_sai_handle->Instance->DR);
            pNodeConfig.DataSize = count * sizeof(int16_t);

            if (HAL_DMAEx_List_BuildNode(&pNodeConfig, &m_sai_dma_nodes[i]) != HAL_OK) {
                return false;
            }

            if (HAL_DMAEx_List_InsertNode_Tail(&m_sai_dma_queue, &m_sai_dma_nodes[i]) != HAL_OK) {
                return false;
            }
        }
        if (HAL_DMAEx_List_ConvertQToDynamic(&m_sai_dma_queue) != HAL_OK) {
            return false;
        }
        return true;
    }

    bool start_audio() {
        if (HAL_DMAEx_List_Init(m_dma_handle) != HAL_OK) {
            return false;
        }
        if (HAL_DMAEx_List_LinkQ(m_dma_handle, &m_sai_dma_queue) != HAL_OK) {
            return false;
        }
        if (HAL_DMAEx_List_Start_IT(m_dma_handle) != HAL_OK) {
            return false;
        }
        if (HAL_SAI_DMAResume(m_sai_handle) != HAL_OK) {
            return false;
        }
        m_sai_handle->State = HAL_SAI_STATE_BUSY_TX;

        return true;
    }

    bool stop_audio() {
        if (HAL_SAI_DMAStop(m_sai_handle) != HAL_OK) {
            return false;
        }
        if (HAL_DMAEx_List_UnLinkQ(m_dma_handle) != HAL_OK) {
            return false;
        }
        if (HAL_DMAEx_List_DeInit(m_dma_handle) != HAL_OK) {
            return false;
        }
        return true;
    }

};
