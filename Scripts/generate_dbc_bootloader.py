#!/usr/bin/env python3
"""
Generate bootloader DBC for STM32-CAN-Bootloader.

This script is the source of truth for the bootloader DBC, similar to the
application workflow in BMS-Firmware-RTOS/Scripts/generate_dbc.py.
"""

from pathlib import Path


def ext_dbc_id(can_id: int) -> int:
    """Encode a 29-bit CAN ID into DBC extended-frame BO_ format."""
    return can_id | 0x80000000


def generate_dbc_lines() -> list[str]:
    lines: list[str] = []

    # Header (match application DBC style)
    lines.append('VERSION ""')
    lines.append('')
    lines.append('NS_ : ')
    lines.append('\tNS_DESC_')
    lines.append('\tCM_')
    lines.append('\tBA_DEF_')
    lines.append('\tBA_')
    lines.append('\tVAL_')
    lines.append('\tCAT_DEF_')
    lines.append('\tCAT_')
    lines.append('\tFILTER')
    lines.append('\tBA_DEF_DEF_')
    lines.append('\tEV_DATA_')
    lines.append('\tENVVAR_DATA_')
    lines.append('\tSGTYPE_')
    lines.append('\tSGTYPE_VAL_')
    lines.append('\tBA_DEF_SGTYPE_')
    lines.append('\tBA_SGTYPE_')
    lines.append('\tSIG_TYPE_REF_')
    lines.append('\tVAL_TABLE_')
    lines.append('\tSIG_GROUP_')
    lines.append('\tSIG_VALTYPE_')
    lines.append('\tSIGTYPE_VALTYPE_')
    lines.append('\tBO_TX_BU_')
    lines.append('\tBA_DEF_REL_')
    lines.append('\tBA_REL_')
    lines.append('\tBA_SGTYPE_REL_')
    lines.append('\tSG_MUL_VAL_')
    lines.append('')
    lines.append('BS_:')
    lines.append('')

    lines.append('BU_: CAN_Host Bootloader BMS_Module_0 BMS_Module_1 BMS_Module_2 BMS_Module_3 BMS_Module_4 BMS_Module_5')
    lines.append('')

    # Attributes (match application style)
    lines.append('BA_DEF_ BO_  "VFrameFormat" ENUM  "StandardCAN","ExtendedCAN";')
    lines.append('BA_DEF_ BO_  "GenMsgCycleTime" INT 0 10000;')
    lines.append('BA_DEF_DEF_  "VFrameFormat" "StandardCAN";')
    lines.append('BA_DEF_DEF_  "GenMsgCycleTime" 0;')
    lines.append('')

    # Protocol IDs
    can_id_host_cmd = 0x18000701
    can_id_bl_resp = 0x18000700

    host_cmd_id = ext_dbc_id(can_id_host_cmd)
    bl_resp_id = ext_dbc_id(can_id_bl_resp)

    # Host -> Bootloader command frame
    lines.append(f'BO_ {host_cmd_id} BL_Host_Command: 8 CAN_Host')
    lines.append(' SG_ Command : 0|8@1+ (1,0) [0|255] "" Bootloader')
    lines.append(' SG_ Byte0 : 8|8@1+ (1,0) [0|255] "" Bootloader')
    lines.append(' SG_ Byte1 : 16|8@1+ (1,0) [0|255] "" Bootloader')
    lines.append(' SG_ Byte2 : 24|8@1+ (1,0) [0|255] "" Bootloader')
    lines.append(' SG_ Byte3 : 32|8@1+ (1,0) [0|255] "" Bootloader')
    lines.append(' SG_ Byte4 : 40|8@1+ (1,0) [0|255] "" Bootloader')
    lines.append(' SG_ Byte5 : 48|8@1+ (1,0) [0|255] "" Bootloader')
    lines.append(' SG_ Byte6 : 56|8@1+ (1,0) [0|255] "" Bootloader')
    lines.append('')

    # Bootloader -> Host response frame
    lines.append(f'BO_ {bl_resp_id} BL_Response: 8 Bootloader')
    lines.append(' SG_ ResponseCode M : 0|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ AckLastError m16 : 8|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ NackErrorCode m17 : 8|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ ErrorCode m18 : 8|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ BusyState m19 : 8|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ ReadyCode1 m20 : 8|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ ReadyCode2 m20 : 16|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ HeartbeatState m20 : 24|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ HeartbeatLastError m20 : 32|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ HeartbeatActiveBankB m20 : 40|1@1+ (1,0) [0|1] "" CAN_Host')
    lines.append(' SG_ HeartbeatBankAValid m20 : 41|1@1+ (1,0) [0|1] "" CAN_Host')
    lines.append(' SG_ HeartbeatBankBValid m20 : 42|1@1+ (1,0) [0|1] "" CAN_Host')
    lines.append(' SG_ HeartbeatMetadataReady m20 : 43|1@1+ (1,0) [0|1] "" CAN_Host')
    lines.append(' SG_ HeartbeatCanCommandSeen m20 : 44|1@1+ (1,0) [0|1] "" CAN_Host')
    lines.append(' SG_ HeartbeatImageInfoValid m20 : 45|1@1+ (1,0) [0|1] "" CAN_Host')
    lines.append(' SG_ HeartbeatVerifiedBankValid m20 : 46|1@1+ (1,0) [0|1] "" CAN_Host')
    lines.append(' SG_ HeartbeatJumpPending m20 : 47|1@1+ (1,0) [0|1] "" CAN_Host')
    lines.append(' SG_ HeartbeatBytesWrittenHi m20 : 48|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ HeartbeatBytesWrittenLo m20 : 56|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ DataByte1 m21 : 8|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ DataByte2 m21 : 16|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ DataByte3 m21 : 24|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ DataByte4 m21 : 32|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ DataByte5 m21 : 40|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ DataByte6 m21 : 48|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ DataByte7 m21 : 56|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ JumpStackPtrMsb m22 : 8|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ JumpStackPtrByte2 m22 : 16|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ JumpStackPtrByte1 m22 : 24|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ JumpStackPtrLsb m22 : 32|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ JumpResetVecByte2 m22 : 40|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ JumpResetVecByte1 m22 : 48|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ JumpResetVecLsb m22 : 56|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append('')

    # Reset command IDs (host -> module), one per module like application DBC style
    reset_ids: list[int] = []
    for module in range(6):
        can_id = 0x08F00F02 + (module << 12)
        dbc_id = ext_dbc_id(can_id)
        reset_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Reset_Command_{module}: 1 CAN_Host')
        lines.append(f' SG_ Reset_Request : 0|8@1+ (1,0) [0|255] "" BMS_Module_{module}')
        lines.append('')

    # Comments
    lines.append(f'CM_ BO_ {host_cmd_id} "Host command frame to bootloader. ID 0x18000701 (extended).";')
    lines.append(f'CM_ BO_ {bl_resp_id} "Bootloader response frame. ID 0x18000700 (extended).";')
    for module, dbc_id in enumerate(reset_ids):
        lines.append(
            f'CM_ BO_ {dbc_id} "Reset command for module {module}. Base ID 0x08F00F02 with module in bits [15:12] (extended).";'
        )
    lines.append('')

    lines.append(
        f'CM_ SG_ {host_cmd_id} Command "ERASE(1), WRITE_FLASH(2), READ_FLASH(3), JUMP(4), STATUS(5), SET_ADDRESS(6), WRITE_DATA(7), GET_ACTIVE_BANK(8), SET_IMAGE_INFO(9), VERIFY_BANK(10).";'
    )
    lines.append(
        f'CM_ SG_ {bl_resp_id} ResponseCode "ACK(16), NACK(17), ERROR(18), BUSY(19), READY(20), DATA(21), JUMP_INFO(22).";'
    )
    lines.append(f'CM_ SG_ {bl_resp_id} AckLastError "ACK payload byte 1: last error code snapshot.";')
    lines.append(f'CM_ SG_ {bl_resp_id} NackErrorCode "NACK payload byte 1: error code for the rejected command.";')
    lines.append(f'CM_ SG_ {bl_resp_id} ErrorCode "ERROR payload byte 1.";')
    lines.append(f'CM_ SG_ {bl_resp_id} BusyState "BUSY payload byte 1: current bootloader state.";')
    lines.append(f'CM_ SG_ {bl_resp_id} ReadyCode1 "READY payload byte 1: version major or special marker.";')
    lines.append(f'CM_ SG_ {bl_resp_id} ReadyCode2 "READY payload byte 2: version minor or special marker.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatState "Bootloader state: IDLE/ERASING/WRITING/READING/VERIFYING/JUMPING.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatLastError "Last bootloader error code.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatActiveBankB "1 = active bank is B, 0 = active bank is A.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatBankAValid "1 = metadata marks Bank A as valid.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatBankBValid "1 = metadata marks Bank B as valid.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatMetadataReady "1 = boot metadata read/initialized successfully.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatCanCommandSeen "1 = at least one CAN command has been received since boot.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatImageInfoValid "1 = image CRC/size info has been provided by host.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatVerifiedBankValid "1 = a bank has been CRC-verified in this session.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatJumpPending "1 = jump to application is pending.";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatBytesWrittenHi "bytes_written[15:8].";')
    lines.append(f'CM_ SG_ {bl_resp_id} HeartbeatBytesWrittenLo "bytes_written[7:0].";')
    lines.append(f'CM_ SG_ {bl_resp_id} DataByte1 "DATA payload byte 1 (command-dependent).";')
    lines.append(f'CM_ SG_ {bl_resp_id} DataByte2 "DATA payload byte 2 (command-dependent).";')
    lines.append(f'CM_ SG_ {bl_resp_id} DataByte3 "DATA payload byte 3 (command-dependent).";')
    lines.append(f'CM_ SG_ {bl_resp_id} DataByte4 "DATA payload byte 4 (command-dependent).";')
    lines.append(f'CM_ SG_ {bl_resp_id} DataByte5 "DATA payload byte 5 (command-dependent).";')
    lines.append(f'CM_ SG_ {bl_resp_id} DataByte6 "DATA payload byte 6 (command-dependent).";')
    lines.append(f'CM_ SG_ {bl_resp_id} DataByte7 "DATA payload byte 7 (command-dependent).";')
    lines.append(f'CM_ SG_ {bl_resp_id} JumpStackPtrMsb "JUMP_INFO payload byte 1: stack pointer[31:24].";')
    lines.append(f'CM_ SG_ {bl_resp_id} JumpStackPtrByte2 "JUMP_INFO payload byte 2: stack pointer[23:16].";')
    lines.append(f'CM_ SG_ {bl_resp_id} JumpStackPtrByte1 "JUMP_INFO payload byte 3: stack pointer[15:8].";')
    lines.append(f'CM_ SG_ {bl_resp_id} JumpStackPtrLsb "JUMP_INFO payload byte 4: stack pointer[7:0].";')
    lines.append(f'CM_ SG_ {bl_resp_id} JumpResetVecByte2 "JUMP_INFO payload byte 5: reset vector[23:16].";')
    lines.append(f'CM_ SG_ {bl_resp_id} JumpResetVecByte1 "JUMP_INFO payload byte 6: reset vector[15:8].";')
    lines.append(f'CM_ SG_ {bl_resp_id} JumpResetVecLsb "JUMP_INFO payload byte 7: reset vector[7:0].";')
    lines.append('')

    # Value tables
    lines.append(
        f'VAL_ {host_cmd_id} Command 1 "CMD_ERASE_FLASH" 2 "CMD_WRITE_FLASH" 3 "CMD_READ_FLASH" 4 "CMD_JUMP_TO_APP" 5 "CMD_GET_STATUS" 6 "CMD_SET_ADDRESS" 7 "CMD_WRITE_DATA" 8 "CMD_GET_ACTIVE_BANK" 9 "CMD_SET_IMAGE_INFO" 10 "CMD_VERIFY_BANK" ;'
    )
    lines.append('')
    lines.append(
        f'VAL_ {bl_resp_id} ResponseCode 16 "RESP_ACK" 17 "RESP_NACK" 18 "RESP_ERROR" 19 "RESP_BUSY" 20 "RESP_READY" 21 "RESP_DATA" 22 "RESP_JUMP_INFO" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatState 0 "BL_STATE_IDLE" 1 "BL_STATE_ERASING" 2 "BL_STATE_WRITING" 3 "BL_STATE_READING" 4 "BL_STATE_VERIFYING" 5 "BL_STATE_JUMPING" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatLastError 0 "ERR_NONE" 1 "ERR_INVALID_COMMAND" 2 "ERR_INVALID_ADDRESS" 3 "ERR_FLASH_ERASE_FAILED" 4 "ERR_FLASH_WRITE_FAILED" 5 "ERR_INVALID_DATA_LENGTH" 6 "ERR_NO_VALID_APP" 7 "ERR_TIMEOUT" 8 "ERR_CRC_MISMATCH" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatActiveBankB 0 "BANK_A_ACTIVE" 1 "BANK_B_ACTIVE" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatBankAValid 0 "BANK_A_INVALID" 1 "BANK_A_VALID" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatBankBValid 0 "BANK_B_INVALID" 1 "BANK_B_VALID" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatMetadataReady 0 "METADATA_NOT_READY" 1 "METADATA_READY" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatCanCommandSeen 0 "NO_COMMAND_SEEN" 1 "COMMAND_SEEN" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatImageInfoValid 0 "IMAGE_INFO_INVALID" 1 "IMAGE_INFO_VALID" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatVerifiedBankValid 0 "VERIFIED_BANK_INVALID" 1 "VERIFIED_BANK_VALID" ;'
    )
    lines.append(
        f'VAL_ {bl_resp_id} HeartbeatJumpPending 0 "JUMP_NOT_PENDING" 1 "JUMP_PENDING" ;'
    )
    lines.append('')

    # Extended frame markers (same style as application DBC)
    lines.append('// Extended CAN ID markers')
    lines.append(f'BA_ "VFrameFormat" BO_ {host_cmd_id} 1;')
    lines.append(f'BA_ "VFrameFormat" BO_ {bl_resp_id} 1;')
    for dbc_id in reset_ids:
        lines.append(f'BA_ "VFrameFormat" BO_ {dbc_id} 1;')

    return lines


def write_dbc(output_path: Path) -> None:
    output_path.write_text("\n".join(generate_dbc_lines()) + "\n", encoding="utf-8")


if __name__ == "__main__":
    repo_root = Path(__file__).resolve().parent.parent
    out_file = repo_root / "STM32L432_Bootloader.dbc"
    write_dbc(out_file)
    print(f"Generated: {out_file}")
