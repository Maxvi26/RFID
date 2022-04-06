#include "mcc_generated_files/mcc.h"

#define nullptr NULL //pointeur de valeur nulle

typedef enum
{
        CommandReg				= 0x01 << 1,	// starts and stops command execution
		ComIEnReg				= 0x02 << 1,	// enable and disable interrupt request control bits
		DivIEnReg				= 0x03 << 1,	// enable and disable interrupt request control bits
		ComIrqReg				= 0x04 << 1,	// interrupt request bits
		DivIrqReg				= 0x05 << 1,	// interrupt request bits
		ErrorReg				= 0x06 << 1,	// error bits showing the error status of the last command executed 
		Status1Reg				= 0x07 << 1,	// communication status bits
		Status2Reg				= 0x08 << 1,	// receiver and transmitter status bits
		FIFODataReg				= 0x09 << 1,	// input and output of 64 byte FIFO buffer
		FIFOLevelReg			= 0x0A << 1,	// number of bytes stored in the FIFO buffer
		WaterLevelReg			= 0x0B << 1,	// level for FIFO underflow and overflow warning
		ControlReg				= 0x0C << 1,	// miscellaneous control registers
		BitFramingReg			= 0x0D << 1,	// adjustments for bit-oriented frames
		CollReg					= 0x0E << 1,	// bit position of the first bit-collision detected on the RF interface
		//						  0x0F			// reserved for future use
		
		// Page 1: Command
		// 						  0x10			// reserved for future use
		ModeReg					= 0x11 << 1,	// defines general modes for transmitting and receiving 
		TxModeReg				= 0x12 << 1,	// defines transmission data rate and framing
		RxModeReg				= 0x13 << 1,	// defines reception data rate and framing
		TxControlReg			= 0x14 << 1,	// controls the logical behavior of the antenna driver pins TX1 and TX2
		TxASKReg				= 0x15 << 1,	// controls the setting of the transmission modulation
		TxSelReg				= 0x16 << 1,	// selects the internal sources for the antenna driver
		RxSelReg				= 0x17 << 1,	// selects internal receiver settings
		RxThresholdReg			= 0x18 << 1,	// selects thresholds for the bit decoder
		DemodReg				= 0x19 << 1,	// defines demodulator settings
		// 						  0x1A			// reserved for future use
		// 						  0x1B			// reserved for future use
		MfTxReg					= 0x1C << 1,	// controls some MIFARE communication transmit parameters
		MfRxReg					= 0x1D << 1,	// controls some MIFARE communication receive parameters
		// 						  0x1E			// reserved for future use
		SerialSpeedReg			= 0x1F << 1,	// selects the speed of the serial UART interface
		
		// Page 2: Configuration
		// 						  0x20			// reserved for future use
		CRCResultRegH			= 0x21 << 1,	// shows the MSB and LSB values of the CRC calculation
		CRCResultRegL			= 0x22 << 1,
		// 						  0x23			// reserved for future use
		ModWidthReg				= 0x24 << 1,	// controls the ModWidth setting?
		// 						  0x25			// reserved for future use
		RFCfgReg				= 0x26 << 1,	// configures the receiver gain
		GsNReg					= 0x27 << 1,	// selects the conductance of the antenna driver pins TX1 and TX2 for modulation 
		CWGsPReg				= 0x28 << 1,	// defines the conductance of the p-driver output during periods of no modulation
		ModGsPReg				= 0x29 << 1,	// defines the conductance of the p-driver output during periods of modulation
		TModeReg				= 0x2A << 1,	// defines settings for the internal timer
		TPrescalerReg			= 0x2B << 1,	// the lower 8 bits of the TPrescaler value. The 4 high bits are in TModeReg.
		TReloadRegH				= 0x2C << 1,	// defines the 16-bit timer reload value
		TReloadRegL				= 0x2D << 1,
		TCounterValueRegH		= 0x2E << 1,	// shows the 16-bit timer value
		TCounterValueRegL		= 0x2F << 1,
		
		// Page 3: Test Registers
		// 						  0x30			// reserved for future use
		TestSel1Reg				= 0x31 << 1,	// general test signal configuration
		TestSel2Reg				= 0x32 << 1,	// general test signal configuration
		TestPinEnReg			= 0x33 << 1,	// enables pin output driver on pins D1 to D7
		TestPinValueReg			= 0x34 << 1,	// defines the values for D1 to D7 when it is used as an I/O bus
		TestBusReg				= 0x35 << 1,	// shows the status of the internal test bus
		AutoTestReg				= 0x36 << 1,	// controls the digital self-test
		VersionReg				= 0x37 << 1,	// shows the software version
		AnalogTestReg			= 0x38 << 1,	// controls the pins AUX1 and AUX2
		TestDAC1Reg				= 0x39 << 1,	// defines the test value for TestDAC1
		TestDAC2Reg				= 0x3A << 1,	// defines the test value for TestDAC2
		TestADCReg				= 0x3B << 1		// shows the value of ADC I and Q channels
		// 						  0x3C			// reserved for production tests
		// 						  0x3D			// reserved for production tests
		// 						  0x3E			// reserved for production tests
		// 						  0x3F			// reserved for production tests
}PCD_Register;

typedef enum
{
    PCD_Idle				= 0x00,		// no action, cancels current command execution
		PCD_Mem					= 0x01,		// stores 25 bytes into the internal buffer
		PCD_GenerateRandomID	= 0x02,		// generates a 10-byte random ID number
		PCD_CalcCRC				= 0x03,		// activates the CRC coprocessor or performs a self-test
		PCD_Transmit			= 0x04,		// transmits data from the FIFO buffer
		PCD_NoCmdChange			= 0x07,		// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
		PCD_Receive				= 0x08,		// activates the receiver circuits
		PCD_Transceive 			= 0x0C,		// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
		PCD_MFAuthent 			= 0x0E,		// performs the MIFARE standard authentication as a reader
		PCD_SoftReset			= 0x0F		// resets the MFRC522
}PCD_Command;

typedef enum
{
        STATUS_OK				,	// Success
        STATUS_ERROR			,	// Error in communication
		STATUS_COLLISION		,	// Collission detected
		STATUS_TIMEOUT			,	// Timeout in communication.
		STATUS_NO_ROOM			,	// A buffer is not big enough.
		STATUS_INTERNAL_ERROR	,	// Internal error in the code. Should not happen ;-)
		STATUS_INVALID			,	// Invalid argument.
		STATUS_CRC_WRONG		,	// The CRC_A does not match
		STATUS_MIFARE_NACK		= 0xff
}StatusCode;
typedef enum
{
        RxGain_18dB				= 0x00 << 4,	// 000b - 18 dB, minimum
		RxGain_23dB				= 0x01 << 4,	// 001b - 23 dB
		RxGain_18dB_2			= 0x02 << 4,	// 010b - 18 dB, it seems 010b is a duplicate for 000b
		RxGain_23dB_2			= 0x03 << 4,	// 011b - 23 dB, it seems 011b is a duplicate for 001b
		RxGain_33dB				= 0x04 << 4,	// 100b - 33 dB, average, and typical default
		RxGain_38dB				= 0x05 << 4,	// 101b - 38 dB
		RxGain_43dB				= 0x06 << 4,	// 110b - 43 dB
		RxGain_48dB				= 0x07 << 4,	// 111b - 48 dB, maximum
		RxGain_min				= 0x00 << 4,	// 000b - 18 dB, minimum, convenience for RxGain_18dB
		RxGain_avg				= 0x04 << 4,	// 100b - 33 dB, average, convenience for RxGain_33dB
		RxGain_max				= 0x07 << 4		// 111b - 48 dB, maximum, convenience for RxGain_48dB
}PCD_RxGain;
typedef enum 
{
        PICC_CMD_REQA			= 0x26,		// REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
		PICC_CMD_WUPA			= 0x52,		// Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
		PICC_CMD_CT				= 0x88,		// Cascade Tag. Not really a command, but used during anti collision.
		PICC_CMD_SEL_CL1		= 0x93,		// Anti collision/Select, Cascade Level 1
		PICC_CMD_SEL_CL2		= 0x95,		// Anti collision/Select, Cascade Level 2
		PICC_CMD_SEL_CL3		= 0x97,		// Anti collision/Select, Cascade Level 3
		PICC_CMD_HLTA			= 0x50,		// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
		PICC_CMD_RATS           = 0xE0,     // Request command for Answer To Reset.
		// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
		// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
		// The read/write commands can also be used for MIFARE Ultralight.
		PICC_CMD_MF_AUTH_KEY_A	= 0x60,		// Perform authentication with Key A
		PICC_CMD_MF_AUTH_KEY_B	= 0x61,		// Perform authentication with Key B
		PICC_CMD_MF_READ		= 0x30,		// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
		PICC_CMD_MF_WRITE		= 0xA0,		// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
		PICC_CMD_MF_DECREMENT	= 0xC0,		// Decrements the contents of a block and stores the result in the internal data register.
		PICC_CMD_MF_INCREMENT	= 0xC1,		// Increments the contents of a block and stores the result in the internal data register.
		PICC_CMD_MF_RESTORE		= 0xC2,		// Reads the contents of a block into the internal data register.
		PICC_CMD_MF_TRANSFER	= 0xB0,		// Writes the contents of the internal data register to a block.
		// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
		// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
		PICC_CMD_UL_WRITE		= 0xA2		
}PICC_Command;

typedef struct {
		uint8_t		size;			// Number of bytes in the UID. 4, 7 or 10.
		uint8_t		uidByte[10];
		uint8_t		sak;			// The SAK (Select acknowledge) byte returned from the PICC after successful selection.
	} Uid;

Uid uid;
    
void Timer2_ISR(void);
void PCD_init(void);
void PCD_Write_Register(PCD_Register registre,uint8_t valeur);
uint8_t PCD_Read_Register(PCD_Register registre);
StatusCode PCD_CommunicateWithPICC(uint8_t command,uint8_t waitIRq,uint8_t *sendData,uint8_t sendLen,uint8_t *backData,uint8_t *backLen,uint8_t *validBits,uint8_t RxAlign,bool checkCRC);
void PCD_Write_Register_block(PCD_Register registre,uint8_t count,uint8_t *valeurs);
void PCD_SetRegisterBitMask(PCD_Register registre,uint8_t mask);
void PCD_ClearRegisterBitMask(PCD_Register reg,uint8_t mask);
void PCD_Read_Register_block(PCD_Register registre,uint8_t count,uint8_t *values,uint8_t RXALIGN);
StatusCode PCD_CalculateCRC(uint8_t *data,uint8_t length,uint8_t *result);
StatusCode PCD_TransceiveData(uint8_t *sendData,uint8_t sendLen,uint8_t *backData,uint8_t *backLen,uint8_t *validBits,uint8_t rxAlign,bool checkCRC);
StatusCode PICC_REQA_or_WUPA(uint8_t command,uint8_t *bufferATQA,uint8_t *bufferSize);
StatusCode PICC_RequestA(uint8_t *bufferATQA,uint8_t *bufferSize);
bool PICC_IsNewCardPresent(void);
//StatusCode PICC_Select(Uid *uid, uint8_t validBits);
bool PICC_ReadCardSerial(void);
void PCD_SetAntennaGain(uint8_t mask);
uint8_t PCD_GetAntennaGain(void);

volatile uint32_t CurrentTime;
uint32_t EntryTime;

void main(void)
{
    SYSTEM_Initialize();
    
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    TMR2_SetInterruptHandler(* Timer2_ISR);
    PCD_init();
    while (1)
    {
        if ( PICC_IsNewCardPresent()) 
        {
            printf("carte presente\r\n");
        }
        
    }
}
/**
 End of File
*/
void PCD_init(void)
{
    uint8_t mask = 0x07<<4;
    SPI1_Open(SPI1_DEFAULT);
    
    RST_SetLow();
    __delay_us(2);
    RST_SetHigh();
    __delay_ms(50);
    
    NSS_SetHigh();
    PCD_Write_Register(TxModeReg,0x00);
    PCD_Write_Register(RxModeReg,0x00);
    PCD_Write_Register(ModWidthReg,0x26);
    PCD_Write_Register(TModeReg,0x80);
    PCD_Write_Register(TPrescalerReg,0xA9);
    PCD_Write_Register(TReloadRegH,0x03);
    PCD_Write_Register(TReloadRegL,0xE8);
    PCD_Write_Register(TxASKReg,0x40);
    PCD_Write_Register(ModeReg,0x3D);
    //PCD_Write_Register(MfRxReg,0x00); parité
   
    
    uint8_t value = PCD_Read_Register(TxControlReg);
    if((value&0x03)!=(0x03)) // Antenne en marche si elle ne l'est pas
    {
        PCD_Write_Register(TxControlReg,value|0x03);
       
    }
    else{}
    PCD_SetAntennaGain(mask);
}

void PCD_Write_Register(PCD_Register registre,uint8_t valeur)
{
    NSS_SetLow();
    SPI1_ExchangeByte(registre); //ecriture
    SPI1_ExchangeByte(valeur);
    NSS_SetHigh();
    
}
uint8_t PCD_Read_Register(PCD_Register registre)
{
    uint8_t valeur;
    NSS_SetLow();
    SPI1_ExchangeByte(0x80 | registre); //lecture
    valeur = SPI1_ExchangeByte(0);
    NSS_SetHigh();
    return valeur;
}
StatusCode PCD_CommunicateWithPICC(uint8_t command,uint8_t waitIRq,uint8_t *sendData,uint8_t sendLen,uint8_t *backData,uint8_t *backLen,uint8_t *validBits,uint8_t rxAlign,bool checkCRC)
{
    uint8_t txLastBits = validBits ? *validBits : 0; //if(validBits){txLastBits=*validBits;}else{txLastBits=0}
    uint8_t bitFraming = (rxAlign << 4) + txLastBits;
    PCD_Write_Register(CommandReg, PCD_Idle);
    PCD_Write_Register(ComIrqReg, 0x7F); //Clear all seven interrupt request bits 
    PCD_Write_Register(FIFOLevelReg, 0x80);	
    PCD_Write_Register_block(FIFODataReg, sendLen, sendData);
    PCD_Write_Register(BitFramingReg, bitFraming);
    PCD_Write_Register(CommandReg, command);
    if (command == PCD_Transceive) 
    {
		PCD_SetRegisterBitMask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
	}
    bool completed = false;
    uint8_t n;
    uint32_t CurrentTimeCopy;
    INTERRUPT_GlobalInterruptDisable();
    EntryTime = CurrentTime;
    INTERRUPT_GlobalInterruptEnable();
    
    do
    {
        n = PCD_Read_Register(ComIrqReg);
        INTERRUPT_GlobalInterruptDisable();
        CurrentTimeCopy = CurrentTime;
        INTERRUPT_GlobalInterruptEnable();
        if(n&waitIRq)
        {
            completed = true;
        }
        if(n&0x01) //temps = 25ms
        {
            return STATUS_TIMEOUT;
        }
         
    }
    while((CurrentTimeCopy-EntryTime)<36);//POUR L'INSTANT LE PROGRAMME NE VA PAS PLUS LOIN
    
    if (!completed) 
    {
        return STATUS_TIMEOUT; 
	}
    uint8_t errorRegValue = PCD_Read_Register(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
	if (errorRegValue & 0x13) 
    {	 // BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}
    uint8_t _validBits = 0;
	
	// If the caller wants data back, get it from the MFRC522.
	if (backData && backLen) 
    {
		uint8_t N = PCD_Read_Register(FIFOLevelReg);	// Number of bytes in the FIFO
		if (N > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = N;											// Number of bytes returned
		PCD_Read_Register_block(FIFODataReg, N, backData, rxAlign);	// Get received data from FIFO
		_validBits = PCD_Read_Register(ControlReg) & 0x07;		// RxLastBits[2:0] indicates the number of valid bits in the last received byte. If this value is 000b, the whole byte is valid.
		if (validBits) 
        {
			*validBits = _validBits;
		}
	}
    if (errorRegValue & 0x08) 
    {		// CollErr
		return STATUS_COLLISION;
	}
	
	// Perform CRC_A validation if requested.
	if (backData && backLen && checkCRC) 
    {
		// In this case a MIFARE Classic NAK is not OK.
		if (*backLen == 1 && _validBits == 4) 
        {
			return STATUS_MIFARE_NACK;
		}
		// We need at least the CRC_A value and all 8 bits of the last byte must be received.
		if (*backLen < 2 || _validBits != 0) 
        {
			return STATUS_CRC_WRONG;
		}
		// Verify CRC_A - do our own calculation and store the control in controlBuffer.
		uint8_t controlBuffer[2];
		StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
		if (status != STATUS_OK) 
        {
			return status;
		}
		if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1])) 
        {
			return STATUS_CRC_WRONG;
		}
	}
	
	return STATUS_OK;
}
void PCD_Write_Register_block(PCD_Register registre,uint8_t count,uint8_t *valeurs)
{
    NSS_SetLow();
    for(uint8_t i=0;i<count;i++)
    {
        PCD_Write_Register(registre,valeurs[i]);
               
    }
    NSS_SetHigh();
}
void PCD_Read_Register_block(PCD_Register registre,uint8_t count,uint8_t *values,uint8_t RXALIGN)
{
    if (count == 0) 
    {
		return;
	}
    uint8_t address = 0x80 | registre;				// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	uint8_t index = 0;		
    NSS_SetLow();
    count--;
    SPI1_ExchangeByte(address);
    if (RXALIGN) {		// Only update bit positions rxAlign..7 in values[0]
		// Create bit mask for bit positions rxAlign..7
		uint8_t mask = (0xFF << RXALIGN) & 0xFF;
		// Read value and tell that we want to read the same address again.
		uint8_t value = PCD_Read_Register(address);
		// Apply mask to both current value of values[0] and the new data in value.
		values[0] = (values[0] & ~mask) | (value & mask);
		index++;
        while (index < count) 
        {
		values[index] = PCD_Read_Register(address);	// Read value and tell that we want to read the same address again.
		index++;
        }
        values[index] = PCD_Read_Register(0);			// Read the final byte. Send 0 to stop reading.
        NSS_SetHigh();	
	}
    
}
void PCD_SetRegisterBitMask(PCD_Register registre,uint8_t mask)
{
    uint8_t tmp;
    tmp = PCD_Read_Register(registre);
    PCD_Write_Register(registre, tmp | mask);
}
void PCD_ClearRegisterBitMask(PCD_Register reg,uint8_t mask)
{
    uint8_t tmp;
	tmp = PCD_Read_Register(reg);
	PCD_Write_Register(reg, tmp & (~mask));
    
}
void Timer2_ISR(void)
{
    CurrentTime++;
}
StatusCode PCD_CalculateCRC(uint8_t *data,uint8_t length,uint8_t *result)
{
    PCD_Write_Register(CommandReg, PCD_Idle);		// Stop any active command.
	PCD_Write_Register(DivIrqReg, 0x04);				// Clear the CRCIRq interrupt request bit
	PCD_Write_Register(FIFOLevelReg, 0x80);			// FlushBuffer = 1, FIFO initialization
	PCD_Write_Register_block(FIFODataReg, length, data);	// Write data to the FIFO
	PCD_Write_Register(CommandReg, PCD_CalcCRC);
    uint32_t CurrentTimeCopy;
    INTERRUPT_GlobalInterruptDisable();
    EntryTime = CurrentTime;
    INTERRUPT_GlobalInterruptEnable();
    do {
		// DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
            INTERRUPT_GlobalInterruptDisable();
            CurrentTimeCopy = CurrentTime;
            INTERRUPT_GlobalInterruptEnable();
            uint8_t n = PCD_Read_Register(DivIrqReg);
            if (n & 0x04) 
            {									// CRCIRq bit set - calculation done
                PCD_Write_Register(CommandReg, PCD_Idle);	// Stop calculating CRC for new content in the FIFO.
                // Transfer the result from the registers to the result buffer
                result[0] = PCD_Read_Register(CRCResultRegL);
                result[1] = PCD_Read_Register(CRCResultRegH);
                return STATUS_OK;
                
            }
       }
    while(CurrentTimeCopy-EntryTime<90);
    return STATUS_TIMEOUT;
}
StatusCode PCD_TransceiveData(uint8_t *sendData,uint8_t sendLen,uint8_t *backData,uint8_t *backLen,uint8_t *validBits,uint8_t rxAlign,bool checkCRC)
{
    uint8_t waitIRq = 0x30;		// RxIRq and IdleIRq
	return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
}
StatusCode PICC_REQA_or_WUPA(uint8_t command,uint8_t *bufferATQA,uint8_t *bufferSize)
{
    uint8_t validBits;
	StatusCode status;
	if (bufferATQA == nullptr || *bufferSize < 2) 
    {	// The ATQA response is 2 bytes long.
		return STATUS_NO_ROOM;
	}
	PCD_ClearRegisterBitMask(CollReg, 0x80);		// ValuesAfterColl=1 => Bits received after collision are cleared.
	validBits = 7;									// For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) byte. TxLastBits = BitFramingReg[2..0]
	status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits,0,false);
	if (status != STATUS_OK) 
    {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) 
    {		// ATQA must be exactly 16 bits.
		return STATUS_ERROR;
	}
    
	return STATUS_OK;
}
StatusCode PICC_RequestA(uint8_t *bufferATQA,uint8_t *bufferSize) 
{
	return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}
bool PICC_IsNewCardPresent(void) 
{
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);
	// Reset baud rates
	PCD_Write_Register(TxModeReg, 0x00);
	PCD_Write_Register(RxModeReg, 0x00);
	// Reset ModWidthReg
	PCD_Write_Register(ModWidthReg, 0x26);
    StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
	return (result == STATUS_OK || result == STATUS_COLLISION);
}
/*StatusCode PICC_Select(Uid *uid, uint8_t validBits)
{
    bool uidComplete;
	bool selectDone;
	bool useCascadeTag;
	uint8_t cascadeLevel = 1;
	StatusCode result;
	uint8_t count;
	uint8_t checkBit;
	uint8_t index;
	uint8_t uidIndex;					// The first index in uid->uidByte[] that is used in the current Cascade Level.
	int8_t currentLevelKnownBits;		// The number of known UID bits in the current Cascade Level.
	uint8_t buffer[9];					// The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
	uint8_t bufferUsed;				// The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
	uint8_t rxAlign;					// Used in BitFramingReg. Defines the bit position for the first bit received.
	uint8_t txLastBits;				// Used in BitFramingReg. The number of valid bits in the last transmitted byte. 
	uint8_t *responseBuffer;
	uint8_t responseLength;
    
    // Sanity checks
	if (validBits > 80) 
    {
		return STATUS_INVALID;
	}
    // Prepare MFRC522
	PCD_ClearRegisterBitMask(CollReg, 0x80);
    
    uidComplete = false;
    while (!uidComplete) 
    {
        switch (cascadeLevel) 
        {
			case 1:
				buffer[0] = PICC_CMD_SEL_CL1;
				uidIndex = 0;
				useCascadeTag = validBits && uid->size > 4;	// When we know that the UID has more than 4 bytes
				break;
                
            case 2:
				buffer[0] = PICC_CMD_SEL_CL2;
				uidIndex = 3;
				useCascadeTag = validBits && uid->size > 7;	// When we know that the UID has more than 7 bytes
				break;
			
			case 3:
				buffer[0] = PICC_CMD_SEL_CL3;
				uidIndex = 6;
				useCascadeTag = false;						// Never used in CL3.
				break;
                
            default:
				return STATUS_INTERNAL_ERROR;
				break;
        }
   
    currentLevelKnownBits = validBits - (8 * uidIndex);
	if (currentLevelKnownBits < 0) 
    {
		currentLevelKnownBits = 0;
    }
	// Copy the known bits from uid->uidByte[] to buffer[]
	index = 2; // destination index in buffer[]
	if (useCascadeTag) 
    {
		buffer[index++] = PICC_CMD_CT;
    }
	uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
    if (bytesToCopy) 
    {
		uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
		if (bytesToCopy > maxBytes) 
        {
			bytesToCopy = maxBytes;
        }
        for (count = 0; count < bytesToCopy; count++) 
        {
				buffer[index++] = uid->uidByte[uidIndex + count];
        }
    }
    // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
	if (useCascadeTag) 
    {
		currentLevelKnownBits += 8;
    }
		
	// Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
	selectDone = false;
    while (!selectDone) 
    {
			// Find out how many bits and bytes to send and receive.
			if (currentLevelKnownBits >= 32) 
            { // All UID bits in this Cascade Level are known. This is a SELECT.
				//Serial.print(F("SELECT: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
				// Calculate BCC - Block Check Character
				buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
				// Calculate CRC_A
				result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
                //printf("c%d",result);
				if (result != STATUS_OK) 
                {
					return result;
				}
				txLastBits		= 0; // 0 => All 8 bits are valid.
				bufferUsed		= 9;
				// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
				responseBuffer	= &buffer[6];
				responseLength	= 3;
			}
			else 
            { // This is an ANTICOLLISION.
				//Serial.print(F("ANTICOLLISION: currentLevelKnownBits=")); Serial.println(currentLevelKnownBits, DEC);
				txLastBits		= currentLevelKnownBits % 8;
				count			= currentLevelKnownBits / 8;	// Number of whole bytes in the UID part.
				index			= 2 + count;					// Number of whole bytes: SEL + NVB + UIDs
				buffer[1]		= (index << 4) + txLastBits;	// NVB - Number of Valid Bits
				bufferUsed		= index + (txLastBits ? 1 : 0);
				// Store response in the unused part of buffer
				responseBuffer	= &buffer[index];
				responseLength	= sizeof(buffer) - index;
			}
			
			// Set bit adjustments
			rxAlign = txLastBits;											// Having a separate variable is overkill. But it makes the next line easier to read.
			PCD_Write_Register(BitFramingReg, (rxAlign << 4) + txLastBits);	// RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]
			
			// Transmit the buffer and receive the response.
			result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign,0);
            printf("r%d",result);
            if (result == STATUS_COLLISION) 
            { // More than one PICC in the field => collision.
                uint8_t valueOfCollReg = PCD_Read_Register(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
				if (valueOfCollReg & 0x20) 
                { // CollPosNotValid
					return STATUS_COLLISION; // Without a valid collision position we cannot continue
				}
				uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
				if (collisionPos == 0) 
                {
					collisionPos = 32;
				}
				if (collisionPos <= currentLevelKnownBits) 
                { // No progress - should not happen 
					return STATUS_INTERNAL_ERROR;
				}
				// Choose the PICC with the bit set.
				currentLevelKnownBits	= collisionPos;
				count			= currentLevelKnownBits % 8; // The bit to modify
				checkBit		= (currentLevelKnownBits - 1) % 8;
				index			= 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
				buffer[index]	|= (1 << checkBit);
			}
			else if (result != STATUS_OK) //on rentre tt le temps ici car timeout
            {
                return result;
			}
            
			else 
            { // STATUS_OK
                //printf("b%d",currentLevelKnownBits);
                if (currentLevelKnownBits >= 32) 
                { // This was a SELECT.
                    selectDone = true; // No more anticollision 
					// We continue below outside the while.
				}
				else 
                { // This was an ANTICOLLISION.
					// We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
					// Run loop again to do the SELECT.
				}
			}
           
		} // End of while (!selectDone)
		// We do not check the CBB - it was constructed by us above.
		// Copy the found UID bytes from buffer[] to uid->uidByte[]
		index			= (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
		bytesToCopy		= (buffer[2] == PICC_CMD_CT) ? 3 : 4;
		for (count = 0; count < bytesToCopy; count++) 
        {
			uid->uidByte[uidIndex + count] = buffer[index++];
		}
		
		// Check response SAK (Select Acknowledge)
		if (responseLength != 3 || txLastBits != 0) 
        { // SAK must be exactly 24 bits (1 byte + CRC_A).
			return STATUS_ERROR;
		}
		// Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
		result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
		if (result != STATUS_OK) 
        {
			return result;
		}
		if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) 
        {
			return STATUS_CRC_WRONG;
		}
		if (responseBuffer[0] & 0x04) 
        { // Cascade bit set - UID not complete yes
			cascadeLevel++;
		}
		else 
        {
			uidComplete = true;
			uid->sak = responseBuffer[0];
		}
	 // End of while (!uidComplete)
	}
	// Set correct uid->size
	uid->size = 3 * cascadeLevel + 1;
    printf("%d",result);
	return STATUS_OK;
}*/

/*bool PICC_ReadCardSerial(void)
{
    StatusCode result = PICC_Select(&uid,0);
	return (result == STATUS_OK);
}*/ 
uint8_t PCD_GetAntennaGain(void) 
{
	return PCD_Read_Register(RFCfgReg) & (0x07<<4);
}
void PCD_SetAntennaGain(uint8_t mask) 
{
	if (PCD_GetAntennaGain() != mask) 
    {						// only bother if there is a change
		PCD_ClearRegisterBitMask(RFCfgReg, (0x07<<4));		// clear needed to allow 000 pattern
		PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07<<4));	// only set RxGain[2:0] bits
	}
}
