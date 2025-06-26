/*
 * sd_sync.c
 *
 *  Code base created in 2014
 *      Author: Weibo Pan
 *  Moved to it's own driver: 11 Oct 2022
 *      Author: Mark Nolan
 */

#include "shimmer_sd_sync.h"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <Configuration/shimmer_config.h>
#include <TaskList/shimmer_taskList.h>
#include <log_and_stream_externs.h>

#if defined(SHIMMER3)
#include "msp430.h"

#include "../../Shimmer_Driver/RN4X/RN4X.h"
#include "../../shimmer_btsd.h"
#include "../5xx_HAL/hal_RTC.h"
#include "Comms/shimmer_bt_uart.h"
#elif defined(SHIMMER3R)
#include "shimmer_definitions.h"
#endif

uint8_t centerNameStr[MAX_CHARS];
uint8_t center_addr[6];
uint8_t nodeNameStr[MAX_NODES][MAX_CHARS];
uint8_t node_addr[MAX_NODES][6];

uint8_t shortExpFlag;
uint8_t syncNodeCnt, syncNodeNum, syncThis, syncNodeSucc, nReboot, currNodeSucc, cReboot;
uint8_t syncSuccC, syncSuccN, syncCurrNode, syncCurrNodeDone, rcFirstOffsetRxed;
uint8_t rcNodeR10Cnt;
uint32_t firstOutlier, rcWindowC, rcNodeReboot;
uint32_t estLen, estLen3, syncCnt;
uint32_t nodeSucc, nodeSuccFull;
uint64_t myLocalTimeLong, myCenterTimeLong, myTimeDiffLong;
uint64_t myTimeDiffLongMin;
uint64_t myTimeDiffArr[SYNC_TRANS_IN_ONE_COMM];
uint8_t myTimeDiffFlagArr[SYNC_TRANS_IN_ONE_COMM];
uint16_t syncCurrNodeExpire, syncNodeWinExpire;
uint8_t myTimeDiffLongFlag;
uint8_t myTimeDiffLongFlagMin;
uint8_t syncResp[SYNC_PACKET_MAX_SIZE], btSdSyncIsRunning;
uint8_t myTimeDiff[SYNC_PACKET_PAYLOAD_SIZE];
uint8_t iAmSyncCenter;

void (*btStartCb)(void);
void (*btStopCb)(uint8_t);

//TODO figure out how best to do away with the need for externs
#if defined(SHIMMER3)
extern uint8_t all0xff[7U];
#elif defined(SHIMMER3R)
static uint8_t all0xff[7U] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
#endif

void ShimSdSync_init(void (*btStart_cb)(void), void (*btStop_cb)(uint8_t))
{
  btSdSyncIsRunning = 0;

  btStartCb = btStart_cb;
  btStopCb = btStop_cb;

  firstOutlier = 1;

  ShimSdSync_resetSyncVariablesNode();
  ShimSdSync_resetSyncVariablesCenter();
  ShimSdSync_resetSyncNodeArray();
  ShimSdSync_resetSyncRcNodeR10Cnt();
  ShimSdSync_resetSyncVariablesBeforeParseConfig();
  ShimSdSync_resetSyncVariablesDuringSyncStart();
}

void ShimSdSync_resetMyTimeDiff(void)
{
  memset(myTimeDiff, 0xff, sizeof(myTimeDiff));
}

void ShimSdSync_resetMyTimeDiffArrays(void)
{
  memset(myTimeDiffFlagArr, 0xff, sizeof(myTimeDiffFlagArr));
  memset(myTimeDiffArr, 0, sizeof(myTimeDiffArr));
}

void ShimSdSync_resetMyTimeDiffLongMin(void)
{
  myTimeDiffLongMin = 0;
  myTimeDiffLongFlagMin = 0;
}

uint8_t *ShimSdSync_myTimeDiffPtrGet(void)
{
  return &myTimeDiff[0];
}

void ShimSdSync_syncRespSet(uint8_t *args, uint8_t count)
{
  memcpy(syncResp, args, count);
}

uint8_t ShimSdSync_isBtSdSyncRunning(void)
{
  return btSdSyncIsRunning;
}

uint8_t ShimSdSync_syncNodeNumGet(void)
{
  return syncNodeNum;
}

uint8_t *ShimSdSync_syncNodeNamePtrForIndexGet(uint8_t index)
{
  return &nodeNameStr[index][0];
}

uint8_t *ShimSdSync_syncCenterNamePtrGet(void)
{
  return &centerNameStr[0];
}

uint8_t ShimSdSync_rcFirstOffsetRxedGet(void)
{
  return rcFirstOffsetRxed;
}

uint8_t ShimSdSync_syncSuccCenterGet(void)
{
  return syncSuccC;
}

uint8_t ShimSdSync_syncSuccNodeGet(void)
{
  return syncSuccN;
}

uint8_t ShimSdSync_syncCntGet(void)
{
  return syncCnt;
}

void ShimSdSync_saveLocalTime(void)
{
  myLocalTimeLong = getRwcTime();
}

void ShimSdSync_resetSyncRcNodeR10Cnt(void)
{
  rcNodeR10Cnt = 0;
}

void ShimSdSync_resetSyncVariablesBeforeParseConfig(void)
{
  syncNodeNum = 0;
  nodeSuccFull = 0;

  memset(centerNameStr, 0, MAX_CHARS);
  memset(center_addr, 0xFF, sizeof(center_addr));
}

void ShimSdSync_resetSyncVariablesDuringSyncStart(void)
{
  ShimSdSync_resetSyncRcNodeR10Cnt();
  syncCnt = 0;
  syncThis = 0;
  syncSuccC = 0;
  syncSuccN = 0;

  ShimSdSync_resetMyTimeDiffLongMin();
  ShimSdSync_resetMyTimeDiffArrays();
  ShimSdSync_resetMyTimeDiff();

  /* Needs to be reset between sessions so that blue LED goes solid again */
  rcFirstOffsetRxed = 0;

  iAmSyncCenter = 0;
}

void ShimSdSync_resetSyncVariablesCenter(void)
{
  cReboot = 0;
  syncNodeCnt = 0;
  nodeSucc = 0;
  syncCurrNode = 0;
  syncCurrNodeDone = 0;
  currNodeSucc = 0;
}

void ShimSdSync_resetSyncVariablesNode(void)
{
  syncNodeSucc = 0;
  nReboot = 0;
}

void ShimSdSync_resetSyncNodeArray(void)
{
  int node_i;
  for (node_i = 0; node_i < MAX_NODES; node_i++)
  {
    *nodeNameStr[node_i] = '\0';
  }
  memset(node_addr, 0xFF, sizeof(node_addr));
}

uint16_t ShimSdSync_parseSyncEstExpLen(uint8_t estExpLenLsb, uint8_t estExpLenMsb)
{
  uint16_t temp16;
  uint32_t temp32;

  temp32 = (uint32_t) estExpLenLsb;
  temp32 += ((uint32_t) estExpLenMsb) << 8;
  temp16 = (uint16_t) (temp32 & 0xffff);

  ShimSdSync_setSyncEstExpLen(temp32);

  return temp16;
}

/* Usually a value of 5 for a 'Short' estimate trial duration (<1hr), 45 for a
 * 'Medium' estimated trial duration (<3hrs), or 180 for a 'Long' estimated
 * trial duration (>3hrs) */
void ShimSdSync_setSyncEstExpLen(uint32_t est_exp_len)
{
  if (est_exp_len < 10)
  {
    shortExpFlag = 1;
    est_exp_len = 0xffff;
  }
  else
  {
    shortExpFlag = 0;
    if (est_exp_len > 180)
    {
      est_exp_len = 180;
    }
  }
  estLen = est_exp_len * 60;
  estLen3 = estLen / 3;
}

void ShimSdSync_parseSyncNodeNamesFromConfig(uint8_t *storedConfigPtr)
{
  uint8_t i;
  uint8_t byte_l, byte_h;

  while (memcmp(all0xff, storedConfigPtr + NV_NODE0 + syncNodeNum * 6, 6)
      && (syncNodeNum < MAX_NODES))
  {
    memcpy(&node_addr[syncNodeNum][0], storedConfigPtr + NV_NODE0 + syncNodeNum * 6, 6);
    for (i = 0; i < 6; i++)
    {
      byte_h = (node_addr[syncNodeNum][i] >> 4) & 0x0f;
      byte_l = node_addr[syncNodeNum][i] & 0x0f;
      nodeNameStr[syncNodeNum][i * 2] = byte_h + (byte_h > 9 ? 'A' - 10 : '0');
      nodeNameStr[syncNodeNum][i * 2 + 1] = byte_l + (byte_l > 9 ? 'A' - 10 : '0');
    }
    *(nodeNameStr[syncNodeNum] + 12) = 0;
    nodeSuccFull |= ShimSdSync_nodeShift(syncNodeNum);
    syncNodeNum++;
  }
}

void ShimSdSync_parseSyncCenterNameFromConfig(uint8_t *storedConfigPtr)
{
  uint8_t i;
  uint8_t byte_l, byte_h;

  memcpy(center_addr, storedConfigPtr + NV_CENTER, 6);
  for (i = 0; i < 6; i++)
  {
    byte_h = (center_addr[i] >> 4) & 0x0f;
    byte_l = center_addr[i] & 0x0f;
    centerNameStr[i * 2] = byte_h + (byte_h > 9 ? 'A' - 10 : '0');
    centerNameStr[i * 2 + 1] = byte_l + (byte_l > 9 ? 'A' - 10 : '0');
  }
  *(centerNameStr + 12) = 0;
}

void ShimSdSync_parseSyncCenterNameFromCfgFile(uint8_t *storedConfigPtr, char *equals)
{
  uint8_t string_length = 0;
  uint8_t i, pchar[3];

  string_length = strlen(equals);
  if (string_length > MAX_CHARS)
  {
    string_length = MAX_CHARS - 1;
  }
  else if (string_length >= 2)
  {
    string_length -= 2;
  }
  else
  {
    string_length = 0;
  }
  if (string_length == 12)
  {
    memcpy((char *) centerNameStr, equals, string_length);
    *(centerNameStr + string_length) = 0;
    pchar[2] = 0;
    for (i = 0; i < 6; i++)
    {
      pchar[0] = *(centerNameStr + i * 2);
      pchar[1] = *(centerNameStr + i * 2 + 1);
      center_addr[i] = strtoul((char *) pchar, 0, 16);
    }
    memcpy(storedConfigPtr + NV_CENTER, center_addr, 6);
  }
}

void ShimSdSync_parseSyncNodeNameFromCfgFile(uint8_t *storedConfigPtr, char *equals)
{
  uint8_t string_length = 0;
  uint8_t i, pchar[3];

  string_length = strlen(equals);

  if (string_length > MAX_CHARS)
  {
    string_length = MAX_CHARS - 1;
  }
  else if (string_length >= 2)
  {
    string_length -= 2;
  }
  else
  {
    string_length = 0;
  }
  if ((string_length == 12) && (syncNodeNum < MAX_NODES))
  {
    memcpy((char *) nodeNameStr[syncNodeNum], equals, string_length);
    *(nodeNameStr[syncNodeNum] + string_length) = 0;
    pchar[2] = 0;
    for (i = 0; i < 6; i++)
    {
      pchar[0] = *(nodeNameStr[syncNodeNum] + i * 2);
      pchar[1] = *(nodeNameStr[syncNodeNum] + i * 2 + 1);
      node_addr[syncNodeNum][i] = strtoul((char *) pchar, 0, 16);
    }
    memcpy(storedConfigPtr + NV_NODE0 + syncNodeNum * 6, &node_addr[syncNodeNum][0], 6);
    nodeSuccFull |= ShimSdSync_nodeShift(syncNodeNum);
    syncNodeNum++;
  }
}

void ShimSdSync_checkSyncCenterName(void)
{
  if (strlen((char *) centerNameStr) == 0) //if no center is appointed, let this guy be the center
  {
    strcpy((char *) centerNameStr, "000000000000");
  }
}

void ShimSdSync_stop(void)
{
  btSdSyncIsRunning = 0;
  rcFirstOffsetRxed = 0;
  ShimSdSync_CommTimerStop();
  btStopCb(1U);
}

void ShimSdSync_start(uint8_t iAmSyncCenterToSet, uint16_t experimentLengthEstimatedInSecToSet)
{
  btSdSyncIsRunning = 1;

  ShimSdSync_resetSyncVariablesDuringSyncStart();

  iAmSyncCenter = iAmSyncCenterToSet;
  ShimSdSync_setSyncEstExpLen(experimentLengthEstimatedInSecToSet);

  if (SYNC_WINDOW_C < (estLen3 - SYNC_BOOT))
  {
    rcWindowC = SYNC_WINDOW_C;
  }
  else
  {
    rcWindowC = (estLen3 - SYNC_BOOT);
  }

  if (SYNC_NODE_REBOOT < (estLen3 / SYNC_WINDOW_N))
  {
    rcNodeReboot = SYNC_NODE_REBOOT;
  }
  else
  {
    rcNodeReboot = (estLen3 / SYNC_WINDOW_N);
  }

  if (iAmSyncCenter)
  {
    firstOutlier = nodeSuccFull;
  }
  else
  {
    firstOutlier = 1;
  }

  ShimSdSync_CommTimerStart();
}

/* Sync Center only. Sends a single sync packet to a node (T10 = transmit 10
 * bytes) and sets getRcomm = 1 waiting for single byte response from a node.
 * Once received, TASK_RCCENTERR1 is set. The sync packet contains the 8-byte
 * real-world clock value in ticks, LSB order. */
void ShimSdSync_centerT10(void)
{
  uint8_t resPacket[SYNC_PACKET_MAX_SIZE];
  uint16_t packet_length = 0;

  *(resPacket + packet_length++) = SET_SD_SYNC_COMMAND;
  *(resPacket + packet_length++) = shimmerStatus.sensing;
  ShimSdSync_saveLocalTime();
  memcpy(resPacket + packet_length, (uint8_t *) (&myLocalTimeLong), 8);
  packet_length += 8;

  if (BT_SD_SYNC_CRC_MODE)
  {
    calculateCrcAndInsert(BT_SD_SYNC_CRC_MODE, resPacket, packet_length);
    packet_length += BT_SD_SYNC_CRC_MODE;
  }

  ShimBt_writeToTxBufAndSend(resPacket, packet_length, SHIMMER_CMD);
}

/* Sync Center only. Called by TASK_RCCENTERR1 when a center receives a byte
 * back from a node (R1 = received 1 byte) after it has sent a sync packet. If
 * ACK_COMMAND_PROCESSED is received, ?, Else the center sends the next sync
 * packet. */
void ShimSdSync_centerR1(void)
{
  if (syncResp[0] != SYNC_FINISHED)
  {
    //DMA2SZ = 1;
    //if (isBtDeviceRn4678())
    //{
    //    // Clear out the BT UART receive buffer by reading its register
    //    UCA1RXBUF;
    //}
    //DMA2_enable();

    //#if BT_DMA_USED_FOR_RX
    //        setDmaWaitingForResponse(1U);
    //#endif
    /* If status strings are enabled the DMA will already be set to listen
     * for bytes so we only need to set if here if that is not the case */
    //if(!areBtStatusStringsEnabled())
    //{
    //    setDmaWaitingForResponse(1U);
    //}
    ShimSdSync_centerT10();
    syncCurrNodeExpire = syncCurrNode + SYNC_EXTEND * SYNC_FACTOR;
  }
  else
  {
    currNodeSucc = 1;
    if (firstOutlier & ShimSdSync_nodeShift(syncNodeCnt))
    {
      firstOutlier &= ~ShimSdSync_nodeShift(syncNodeCnt);
    }
    else
    {
      nodeSucc |= ShimSdSync_nodeShift(syncNodeCnt);
    }
  }
}

uint32_t ShimSdSync_nodeShift(uint8_t shift_value)
{
  uint32_t sync_node_shift = 0x01;
  sync_node_shift <<= shift_value;
  return sync_node_shift;
}

/* Sync Node only. Called by TASK_RCNODER10 after a node has received a 10 byte
 * sync packet (R10 = received 10 bytes) from a center */
void ShimSdSync_nodeR10(void)
{
  uint8_t nodeResponse;

  //only nodes do this
  if (!BT_SD_SYNC_CRC_MODE
      || checkCrc(BT_SD_SYNC_CRC_MODE, &syncResp[0], SYNC_PACKET_SIZE_CMD + SYNC_PACKET_PAYLOAD_SIZE))
  { //if received the correct 6 bytes:
    uint8_t sd_tolog;
    sd_tolog = syncResp[SYNC_PACKET_FLG_IDX];
    myCenterTimeLong = *(uint64_t *) (syncResp + SYNC_PACKET_TIME_IDX); //get myCenterTimeLong

    if (myLocalTimeLong > myCenterTimeLong)
    {
      myTimeDiffLongFlag = 0;
      myTimeDiffLong = myLocalTimeLong - myCenterTimeLong;
    }
    else
    {
      myTimeDiffLongFlag = 1;
      myTimeDiffLong = myCenterTimeLong - myLocalTimeLong;
    }
    myTimeDiffArr[rcNodeR10Cnt] = myTimeDiffLong;
    myTimeDiffFlagArr[rcNodeR10Cnt] = myTimeDiffLongFlag;

    memset(syncResp, 0, SYNC_PACKET_MAX_SIZE);

    if (rcNodeR10Cnt++ < (SYNC_TRANS_IN_ONE_COMM - 1))
    {
      nodeResponse = SYNC_PACKET_RESEND;
    }
    else
    {
      if (ShimConfig_getStoredConfig()->singleTouchStart && !shimmerStatus.sensing && sd_tolog)
      {
        ShimTask_set(TASK_STARTSENSING);
      }
      syncNodeSucc = 1;
      if (!firstOutlier)
      {
        rcFirstOffsetRxed = 1;
        ShimSdSync_rcFindSmallest();
        myTimeDiff[0] = myTimeDiffLongFlagMin;
        memcpy(myTimeDiff + 1, (uint8_t *) &myTimeDiffLongMin, 8);
      }
      ShimSdSync_resetMyTimeDiffLongMin();
      ShimSdSync_resetSyncRcNodeR10Cnt();
      nodeResponse = SYNC_FINISHED;
    }
  }
  else
  {
    nodeResponse = SYNC_PACKET_RESEND;
  }

  //#if BT_DMA_USED_FOR_RX
  //    if(nodeResponse == SYNC_PACKET_RESEND)
  //    {
  //        /* If status strings are enabled the DMA will already be set to listen
  //         * for bytes so we only need to set if here if that is not the case */
  //        if(!areBtStatusStringsEnabled())
  //        {
  ////            setDmaWaitingForResponse(10U);
  //setDmaWaitingForResponse(1U);
  //}
  //}
  //#endif

  memset(syncResp, 0, sizeof(syncResp));
  ShimSdSync_nodeT1(nodeResponse);
}

/* Sync Node only. For responding to Center node after having received a single
 * packet. 1 is passed in if ready to receive the next 10 byte sync packet.
 * ACK_COMMAND_PROCESSED is passed in if 50 packets have been received */
void ShimSdSync_nodeT1(uint8_t val)
{
  uint8_t syncResponse[] = { ACK_COMMAND_PROCESSED, SD_SYNC_RESPONSE, val };
  ShimBt_writeToTxBufAndSend(&syncResponse[0], 3U, SHIMMER_CMD);
  if (syncNodeWinExpire < (syncCnt + SYNC_EXTEND * SYNC_FACTOR))
  {
    syncNodeWinExpire++;
  }
}

/* Sync Node only. Called by a node after it has received all sync packets. */
uint8_t ShimSdSync_rcFindSmallest(void)
{
  uint8_t i, j, k, black_list[20], black_list_cnt = 0, far_cnt = 0, black = 0;
  uint64_t to_compare_val, diff_val;
  for (i = 0; i < SYNC_TRANS_IN_ONE_COMM; i++)
  {
    for (j = 1; j < 6; j++)
    {
      to_compare_val = myTimeDiffArr[(i + j) % SYNC_TRANS_IN_ONE_COMM];
      if (myTimeDiffArr[i] > to_compare_val)
      {
        diff_val = myTimeDiffArr[i] - to_compare_val;
      }
      else
      {
        diff_val = to_compare_val - myTimeDiffArr[i];
      }
      if (diff_val > 3277) //0.1*32768
      {
        far_cnt++;
      }
    }
    if ((far_cnt >= 4) && (black_list_cnt < 20))
    {
      black_list[black_list_cnt++] = i;
    }
    far_cnt = 0;
  }

  ShimSdSync_resetMyTimeDiffLongMin();
  for (i = 0; i < SYNC_TRANS_IN_ONE_COMM; i++)
  {
    for (k = 0; k < black_list_cnt; k++)
    {
      if (i == black_list[k])
      {
        black = 1;
        break;
      }
    }
    if (black)
    {
      black = 0;
      continue;
    }
    if ((!myTimeDiffLongMin) && (!myTimeDiffLongFlagMin))
    {
      myTimeDiffLongFlagMin = myTimeDiffFlagArr[i];
      myTimeDiffLongMin = myTimeDiffArr[i];
    }
    else
    {
      if ((!myTimeDiffFlagArr[i]) && (!myTimeDiffLongFlagMin))
      { //was pos, curr pos
        if (myTimeDiffArr[i] < myTimeDiffLongMin)
        {
          myTimeDiffLongMin = myTimeDiffArr[i];
        }
      }
      else if ((!myTimeDiffFlagArr[i]) && myTimeDiffLongFlagMin)
      { //was neg, curr pos
      }
      else if (myTimeDiffFlagArr[i] && (!myTimeDiffLongFlagMin))
      { //was pos, curr neg
        myTimeDiffLongFlagMin = myTimeDiffFlagArr[i];
        myTimeDiffLongMin = myTimeDiffArr[i];
      }
      else if (myTimeDiffFlagArr[i] && myTimeDiffLongFlagMin)
      { //was neg, curr neg
        if (myTimeDiffArr[i] > myTimeDiffLongMin)
        {
          myTimeDiffLongMin = myTimeDiffArr[i];
        }
      }
    }
  }

  ShimSdSync_resetMyTimeDiffArrays();
  return 0;
}

void ShimSdSync_handleSyncTimerTrigger(void)
{
  if (ShimSdSync_isBtSdSyncRunning())
  {
    if (syncCnt >= estLen3 * SYNC_FACTOR)
    { //there must be: estLen3>SYNC_WINDOW_C
      syncCnt = 0;
      syncSuccN = 0; //reset node success flag
      if (syncThis > 3)
      {
        //can stop syncing after certain #
      }
      else
      {
        syncThis++;
      }
    }
    else
    {
      syncCnt++;
    }

    if (iAmSyncCenter)
    { //i am Center
      ShimSdSync_handleSyncTimerTriggerCenter();
    }
    else
    { //i am Node
      ShimSdSync_handleSyncTimerTriggerNode();
    }
  }
  else //idle: no_RC mode
  {
    if (LogAndStream_isDockedOrUsbIn())
    {
      ShimTask_setStopSensing();
      if (shimmerStatus.btPowerOn)
      {
        btStopCb(0);
      }
    }
  }
}

void ShimSdSync_handleSyncTimerTriggerCenter(void)
{
  if (shimmerStatus.sensing && (syncNodeNum > 0))
  {
    if (syncCnt == 1)
    {
      //start
      ShimSdSync_resetSyncVariablesCenter();
      ShimSdSync_startBtForSync();
    }
    else if ((syncCnt > SYNC_BOOT * SYNC_FACTOR) && (syncCnt < SYNC_WINDOW_C * SYNC_FACTOR))
    {
      //try to connect to each node
      //loop 1:n, x times
      if (nodeSucc != nodeSuccFull)
      {
        if (syncNodeCnt < syncNodeNum)
        {
          syncCurrNode++;
          if (!cReboot)
          {
            if (syncCurrNode == 1)
            {
              while (nodeSucc & ShimSdSync_nodeShift(syncNodeCnt))
              {
                if (++syncNodeCnt >= syncNodeNum)
                {
                  syncNodeCnt = 0;
                }
              }
              //TODO figure how best to handle/locate this required check
              if (shimmerStatus.btIsInitialised)
              {
#if defined(SHIMMER3)
                BT_connect(nodeNameStr[syncNodeCnt]);
#else
                BT_connect(&node_addr[syncNodeCnt][0]);
#endif
              }
              currNodeSucc = 0;
              syncCurrNodeDone = 0;
              syncCurrNodeExpire = SYNC_T_EACHNODE_C * SYNC_FACTOR;
            }
            else if ((syncCurrNode == syncCurrNodeExpire) || currNodeSucc)
            {
              if (currNodeSucc)
              {
                BT_disconnect();
                currNodeSucc = 0;
              }
              btStopCb(0);
              cReboot = 1;
              if (shortExpFlag)
              {
                nodeSucc |= 1 << syncNodeCnt;
              }

              syncNodeCnt++;
            }
            else if (syncCurrNodeDone
                && (syncCurrNode == syncCurrNodeDone + SYNC_CD * SYNC_FACTOR))
            {
              syncCurrNode = 0;
            }
          }
          else
          { //cReboot>0
            if (cReboot == 1)
            {
              cReboot = 2;
              ShimSdSync_startBtForSync();
            }
            else if ((cReboot >= 2) && (cReboot < 5 * SYNC_FACTOR))
            {
              if (shimmerStatus.btPowerOn)
              {
                syncCurrNodeDone = syncCurrNode + SYNC_CD * SYNC_FACTOR - 1;
                cReboot = 0;
              }
              else
              {
                cReboot++;
              }
            }
            else
            {
              btStopCb(0);
              cReboot = 1;
            }
          }
        }
        else
        {
          syncNodeCnt = 0;
        }
      }
      else
      {
        btStopCb(0);
        syncSuccC = 1;
        if (shortExpFlag)
        {
          syncCnt = estLen3 * SYNC_FACTOR;
        }
      }
    }
    else if (syncCnt == SYNC_WINDOW_C * SYNC_FACTOR)
    {
      //power off
      btStopCb(0);
      if (nodeSucc == nodeSuccFull)
      {
        syncSuccC = 1;
      }
      else
      {
        syncSuccC = 0;
      }
      if (shortExpFlag)
      {
        syncCnt = estLen3 * SYNC_FACTOR;
      }
    }
  }
}

void ShimSdSync_handleSyncTimerTriggerNode(void)
{
  if (!syncSuccN)
  {
    if (syncCnt == 1)
    {
      ShimSdSync_resetMyTimeDiffLongMin();
      ShimSdSync_resetSyncVariablesNode();
    }
    else if (((syncCnt < syncNodeWinExpire) && (syncCnt > SYNC_BOOT))
        && (syncCnt != (SYNC_CD * (nReboot + 1) + SYNC_WINDOW_N * nReboot) * SYNC_FACTOR))
    {
      if (syncNodeSucc)
      {
        btStopCb(0);
        syncNodeSucc = 0;
        nReboot = 0;
        if (firstOutlier)
        {
          syncCnt = (SYNC_CD * nReboot + SYNC_WINDOW_N * nReboot) * SYNC_FACTOR;
          firstOutlier = 0;
        }
        else
        {
          syncSuccN = 1;
          if (shortExpFlag)
          {
            syncCnt = estLen3 * SYNC_FACTOR;
          }
          else
          {
            syncCnt = (SYNC_CD * rcNodeReboot + SYNC_WINDOW_N * (SYNC_NEXT2MATCH - 1)) * SYNC_FACTOR;
          }
        }
      }
    }
    else if (syncCnt == syncNodeWinExpire)
    {
      btStopCb(0);
      if (firstOutlier)
      {
        nReboot = 1;
        syncCnt = (SYNC_CD * nReboot + SYNC_WINDOW_N * nReboot) * SYNC_FACTOR;
      }
      else
      {
        if (nReboot < rcNodeReboot)
        {
          nReboot++;
        }
        else
        {
          nReboot = 0;
        }
      }
      syncSuccN = 0;
    }
    else if (syncCnt == (SYNC_CD * (nReboot + 1) + SYNC_WINDOW_N * nReboot) * SYNC_FACTOR)
    {
      ShimSdSync_startBtForSync();
      syncNodeWinExpire
          = (SYNC_CD * (nReboot + 1) + SYNC_WINDOW_N * (nReboot + 1)) * SYNC_FACTOR;
    }
  }
}

void ShimSdSync_startBtForSync(void)
{
#if defined(SHIMMER3)
  BT_init();
  BT_rn4xDisableRemoteConfig(1);
  BT_setUpdateBaudDuringBoot(1);
#endif
  btStartCb();
}

//Timer2:
//ccr1: for blink timer
void ShimSdSync_CommTimerStart(void)
{
#if defined(SHIMMER3)
  TA0CTL = TASSEL_1 + MC_2 + TACLR; //ACLK, continuous mode, clear TAR
  TA0CCTL1 = CCIE;
  TA0CCR1 = GetTA0() + 16384;
#elif defined(SHIMMER3R)
  RTC_setupAndStartSdSyncAlarm();
#endif
  shimmerStatus.sdSyncCommTimerRunning = 1;
}

inline void ShimSdSync_CommTimerStop(void)
{
#if defined(SHIMMER3)
  TA0CTL = MC_0; //StopTb0()
  //rcommStatus=0;
  TA0CCTL1 &= ~CCIE;
#elif defined(SHIMMER3R)
  RTC_stopSdSyncAlarm();
#endif
  shimmerStatus.sdSyncCommTimerRunning = 0;
}

#if defined(SHIMMER3)
inline uint16_t GetTA0(void)
{
  register uint16_t t0, t1;
  uint8_t ie;
  if (ie = (__get_SR_register() & GIE)) //interrupts enabled? // @suppress("Assignment in condition")
  {
    __disable_interrupt();
  }
  t1 = TA0R;
  do
  {
    t0 = t1;
    t1 = TA0R;
  } while (t0 != t1);
  if (ie)
  {
    __enable_interrupt();
  }
  return t1;
}

#pragma vector = TIMER0_A1_VECTOR

__interrupt void TIMER0_A1_ISR(void)
{
  switch (__even_in_range(TA0IV, 14))
  {
    case 0:
      break; //No interrupt
    case 2:  //TA0CCR1
      TA0CCR1 += SYNC_PERIOD;

      /* SDLog handles auto-stop in TIMER0_A1_VECTOR whereas LogAndStream handles it in TIMER0_B1_VECTOR */
      //if (sensing && maxLen)
      //{
      //    if (maxLenCnt < maxLen * SYNC_FACTOR)
      //        maxLenCnt++;
      //    else
      //    {
      //        stopLogging = 1;
      //        //stopSensing = 1;
      //        maxLenCnt = 0;
      //        return;
      //    }
      //}

      ShimSdSync_handleSyncTimerTrigger();

      break;
    case 4:
      break; //TA0CCR2 not used
    case 6:
      break; //Reserved
    case 8:
      break; //Reserved
    case 10:
      break; //Reserved
    case 12:
      break; //Reserved
    case 14:
      break; //TAIFG overflow handler
  }
}
#endif
