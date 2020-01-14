/**
  ******************************************************************************
  * Copyright(c) HUST ARMS 302 All rights reserved. 
  * - Filename:  grid_data_pack_protocol.h
  * - Author:    Zhao Wang
  * - Version:   V1.0.0
  * - Date:      2019/12/3
  * - Brief:     Grid data of global map packing protocol
  ******************************************************************************
*/

#ifndef GRID_DATA_PACK_PROTOCOL_H_
#define GRID_DATA_PACK_PROTOCOL_H_


struct GridDataPackProtocol{
	static const char pack_head_ = 0xA5;
	static const char pack_tail_ = 0xAA;
	static const char pack_sec_bit_ = 0x5A;
};

#endif

