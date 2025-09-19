# STM32-DMA-MemCopy-Verify
Demonstrates memory-to-memory data transfer using DMA in interrupt mode on STM32.  
After transfer completion, the data in the destination buffer is compared with the source buffer, and LEDs indicate whether the transfer was successful.

---

## Features
- **DMA Transfer:** Efficient memory-to-memory data transfer without CPU blocking.  
- **Interrupt Mode:** Callback function executed after DMA transfer completion.  
- **Buffer Comparison:** Custom function to verify data integrity after transfer.  
- **LED Feedback:**  
  - **Green LED:** Data transfer successful (buffers match)  
  - **Red LED:** Data mismatch (buffers differ)  
- **Button Trigger:** DMA transfer starts when a button is pressed.  

---

## Hardware Setup
- **MCU:** STM32F103C8T6 (Blue Pill)  
- **LEDs:**  
  - Green LED → Indicates successful transfer  
  - Red LED → Indicates mismatch  
- **Button:** Trigger DMA transfer (active LOW)  

---

## How It Works

### DMA Configuration
- DMA Channel: `DMA1_Channel1`  
- Direction: Memory to Memory  
- Peripheral & Memory Increment: Enabled  
- Data Alignment: Word (32-bit)  
- Mode: Normal (single transfer)  
- Priority: Low  

### Execution Flow
1. User presses the button → triggers DMA transfer.  
2. `HAL_DMA_Start_IT()` starts transferring `srcBuffer` to `dstBuffer`.  
3. When transfer completes, **TransferComplete callback** is executed.  
4. `BufferCmp()` compares source and destination buffers:  
   - If identical → Green LED turns ON  
   - If different → Red LED turns ON  

### Key Code Snippet
```c
static void TransferCompelete(DMA_HandleTypeDef * hdma){
    if(BufferCmp((uint32_t*)srcBuffer , (uint32_t*)dstBuffer , BUFFER_SIZE)){
        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
    }
    else{
        HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);        
    }
}
```

---

#Applications in Larger Projects

 This simple educational project demonstrates a concept widely used in industrial and high-performance embedded systems:

1. High-Speed Data Acquisition
- DMA can transfer large blocks of sensor data (e.g., camera frames, ADC samples) to memory without blocking the CPU.

2. Data Integrity Verification
- In critical applications (medical devices, industrial control, network communication), transferred data is verified using checksums, CRC, or buffer comparison to detect errors.
- ensures the system reacts correctly to valid data and avoids processing corrupted information.

3. Real-Time Processing
- CPU can execute other tasks (signal processing, communication, control loops) while DMA transfers data in the background.
- Essential for applications requiring low latency and deterministic performance.

4. Embedded Testing and Validation
- Similar techniques are used for testing memory modules, peripherals, and communication channels in production.
- LEDs, logs, or debug outputs indicate transfer success, just like the Green/Red LED feedback in this project.

---

# Notes

- Button uses pull-up, so press connects input to GND.
- Buffer size can be adjusted; ensure DMA configuration and comparison function are updated accordingly.
- For real-world applications, larger data blocks and hardware CRC are often used.
        
