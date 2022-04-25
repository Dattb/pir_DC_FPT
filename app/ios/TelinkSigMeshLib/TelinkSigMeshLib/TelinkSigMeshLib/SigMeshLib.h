/********************************************************************************************************
 * @file     SigMeshLib.h
 *
 * @brief    for TLSR chips
 *
 * @author     telink
 * @date     Sep. 30, 2010
 *
 * @par      Copyright (c) 2010, Telink Semiconductor (Shanghai) Co., Ltd.
 *           All rights reserved.
 *
 *             The information contained herein is confidential and proprietary property of Telink
 *              Semiconductor (Shanghai) Co., Ltd. and is available under the terms
 *             of Commercial License Agreement between Telink Semiconductor (Shanghai)
 *             Co., Ltd. and the licensee in separate contract or the terms described here-in.
 *           This heading MUST NOT be removed from this file.
 *
 *              Licensees are granted free, non-transferable use of the information in this
 *             file under Mutual Non-Disclosure Agreement. NO WARRENTY of ANY KIND is provided.
 *
 *******************************************************************************************************/
//
//  SigMeshLib.h
//  TelinkSigMeshLib
//
//  Created by 梁家誌 on 2019/8/15.
//  Copyright © 2019年 Telink. All rights reserved.
//

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@class SigMessageHandle,SigMeshAddress,SigProxyConfigurationMessage,SDKLibCommand,SigSecureNetworkBeacon,SigNetworkPdu;

@protocol SigMessageDelegate <NSObject>
@optional

/// A callback called whenever a Mesh Message has been received from the mesh network.
/// @param message The received message.
/// @param source The Unicast Address of the Element from which the message was sent.
/// @param destination The address to which the message was sent.
- (void)didReceiveMessage:(SigMeshMessage *)message sentFromSource:(UInt16)source toDestination:(UInt16)destination;

/// A callback called when an unsegmented message was sent to the SigBearer, or when all segments of a segmented message targetting a Unicast Address were acknowledged by the target Node.
/// @param message The message that has been sent.
/// @param localElement The local Element used as a source of this message.
/// @param destination The address to which the message was sent.
- (void)didSendMessage:(SigMeshMessage *)message fromLocalElement:(SigElementModel *)localElement toDestination:(UInt16)destination;

/// A callback called when a message failed to be sent to the target Node, or the respnse for an acknowledged message hasn't been received before the time run out.
/// For unsegmented unacknowledged messages this callback will be invoked when the SigBearer was closed.
/// For segmented unacknowledged messages targetting a Unicast Address, besides that, it may also be called when sending timed out before all of the segments were acknowledged by the target Node, or when the target Node is busy and not able to proceed the message at the moment.
/// For acknowledged messages the callback will be called when the response has not been received before the time set by `acknowledgmentMessageTimeout` run out. The message might have been retransmitted multiple times and might have been received by the target Node. For acknowledged messages sent to a Group or Virtual Address this will be called when the response has not been received from any Node.
/// @param message The message that has failed to be delivered.
/// @param localElement The local Element used as a source of this message.
/// @param destination The address to which the message was sent.
/// @param error The error that occurred.
- (void)failedToSendMessage:(SigMeshMessage *)message fromLocalElement:(SigElementModel *)localElement toDestination:(UInt16)destination error:(NSError *)error;

- (void)didReceiveSigProxyConfigurationMessage:(SigProxyConfigurationMessage *)message sentFromSource:(UInt16)source toDestination:(UInt16)destination;
- (void)didReceiveSigSecureNetworkBeaconMessage:(SigSecureNetworkBeacon *)message;

@end


@interface SigMeshLib : NSObject
/// command list cache.
@property (nonatomic,strong) NSMutableArray <SDKLibCommand *>*commands;
/// A queue to handle incoming and outgoing messages.
@property (nonatomic,strong) dispatch_queue_t queue;
/// A queue to call delegate methods on.
@property (nonatomic,strong) dispatch_queue_t delegateQueue;

/// Mesh Network data.
@property (nonatomic,strong) SigDataSource *dataSource;
/// The sender object should send PDUs created by the manager using any Bearer.
@property (nonatomic,strong) SigBearer *bearer;

/// The delegate will receive callbacks whenever a complete Mesh Message has been received and reassembled.
@property (nonatomic, weak, readonly) id <SigMessageDelegate>delegate;
@property (nonatomic, weak, readwrite) id <SigMessageDelegate>delegateForDeveloper;

#pragma mark - Network Manager properties

/// The Default TTL will be used for sending messages, if the value has not been set in the Provisioner's Node. By default it is set to 10, which is a reasonable value. The TTL shall be in range 2...127.
@property (nonatomic,assign) UInt8 defaultTtl;//10

/// The timeout after which an incomplete segmented message will be abandoned. The timer is restarted each time a segment of this message is received.
///
/// The incomplete timeout should be set to at least 10 seconds.
@property (nonatomic,assign) NSTimeInterval incompleteMessageTimeout;//10.0

/// The amount of time after which the lower transport layer sends a Segment Acknowledgment message after receiving a segment of a multi-segment message where the destination is a Unicast Address of the Provisioner's Node.
///
/// The acknowledgment timer shall be set to a minimum of 150 + 50 * TTL milliseconds. The TTL dependent part is added automatically, and this value shall specify only the constant part.
@property (nonatomic,assign) NSTimeInterval acknowledgmentTimerInterval;//0.150

/// The time within which a Segment Acknowledgment message is expected to be received after a segment of a segmented message has been sent. When the timer is fired, the non-acknowledged segments are repeated, at most `retransmissionLimit` times.
///
/// The transmission timer shall be set to a minimum of 200 + 50 * TTL milliseconds. The TTL dependent part is added automatically, and this value shall specify only the constant part.
///
/// If the SigBearer.share.isProvisioned is NO, it is recommended to set the transmission interval longer than the connection interval, so that the acknowledgment had a chance to be received.
@property (nonatomic,assign) NSTimeInterval transmissionTimerInteral;//0.200

/// Number of times a non-acknowledged segment will be re-send before the message will be cancelled.
///
/// The limit may be decreased with increasing of `transmissionTimerInterval` as the target Node has more time to reply with the Segment Acknowledgment message.
@property (nonatomic,assign) int retransmissionLimit;//5

/// If the Element does not receive a response within a period of time known as the acknowledged message timeout, then the Element may consider the message has not been delivered, without sending any additional messages.
///
/// The `failedToSendMessage:fromLocalElement:toDestination:error:` callback will be called on timeout.
///
/// The acknowledged message timeout should be set to a minimum of 30 seconds.
//@property (nonatomic,assign) NSTimeInterval acknowledgmentMessageTimeout;//30.0

/// The base time after which the acknowledgmed message will be repeated.
///
/// The repeat timer will be set to the base time + 50 * TTL milliseconds + 50 * segment count. The TTL and segment count dependent parts are added automatically, and this value shall specify only the constant part.
//@property (nonatomic,assign) NSTimeInterval acknowledgmentMessageInterval;//2.0

/// 4.2.19.2 Network Transmit Interval Steps
/// - seeAlso: Mesh_v1.0.pdf  (page.151)
@property (nonatomic,assign) UInt8 networkTransmitIntervalSteps;//defount is 0b1111=31.
@property (nonatomic,assign,readonly) double networkTransmitInterval;//defount is (0b1111+1)*10=320ms.

@property (nonatomic,assign) int segmentTXTimeout;//15
@property (nonatomic,assign) int segmentRXTimeout;//15

@property (nonatomic,assign) BOOL isReceiveSegmentPDUing;

+ (instancetype)new __attribute__((unavailable("please initialize by use .share or .share()")));
- (instancetype)init __attribute__((unavailable("please initialize by use .share or .share()")));


+ (SigMeshLib *)share;

#pragma mark - Receive Mesh Messages

/// This method should be called whenever a PDU has been received from the mesh network using SigBearer. When a complete Mesh Message is received and reassembled, the delegate's `didReceiveMessage:sentFromSource:toDestination:` will be called.
/// @param data The PDU received.
/// @param type The PDU type.
- (void)bearerDidDeliverData:(NSData *)data type:(SigPduType)type;

/// This method should be called whenever a PDU has been decoded from the mesh network using SigNetworkLayer.
/// @param networkPdu The network pdu in (Mesh_v1.0.pdf 3.4.4 Network PDU).
- (void)receiveNetworkPdu:(SigNetworkPdu *)networkPdu;

/// This method should be called whenever a PDU has been received from the kOnlineStatusCharacteristicsID.
/// @param address address of node
/// @param state state of node
/// @param bright100 bright of node
/// @param temperature100 temperature of node
- (void)updateOnlineStatusWithDeviceAddress:(UInt16)address deviceState:(DeviceState)state bright100:(UInt8)bright100 temperature100:(UInt8)temperature100;

#pragma mark - Send Mesh Messages

/// Sends Configuration Message to the Node with given destination Address. The `destination` must be a Unicast Address, otherwise the method does nothing.
///
/// A `SigMessageDelegate` method will be called when the message has been sent, delivered, or fail to be sent.
///
/// @param message The message to be sent.
/// @param destination The destination Unicast Address.
/// @param initialTtl The initial TTL (Time To Live) value of the message. initialTtl is Relayed TTL.
/// @param command The command save message and callback.
/// @returns Message handle that can be used to cancel sending.
- (SigMessageHandle *)sendConfigMessage:(SigConfigMessage *)message toDestination:(UInt16)destination withTtl:(UInt8)initialTtl command:(SDKLibCommand *)command;
- (SigMessageHandle *)sendConfigMessage:(SigConfigMessage *)message toDestination:(UInt16)destination command:(SDKLibCommand *)command;

/// Encrypts the message with the Application Key and a Network Key bound to it, and sends to the given destination address.
///
/// A `SigMessageDelegate` method will be called when the message has been sent, delivered, or fail to be sent.
///
/// @param message The message to be sent.
/// @param localElement The source Element. If `nil`, the primary Element will be used. The Element must belong to the local Provisioner's Node.
/// @param destination The destination address.
/// @param initialTtl The initial TTL (Time To Live) value of the message. initialTtl is Relayed TTL.
/// @param applicationKey The Application Key to sign the message.
/// @param command The command save message and callback.
/// @returns Message handle that can be used to cancel sending.
- (SigMessageHandle *)sendMeshMessage:(SigMeshMessage *)message fromLocalElement:(nullable SigElementModel *)localElement toDestination:(SigMeshAddress *)destination withTtl:(UInt8)initialTtl usingApplicationKey:(SigAppkeyModel *)applicationKey command:(SDKLibCommand *)command;
- (SigMessageHandle *)sendMeshMessage:(SigMeshMessage *)message fromLocalElement:(nullable SigElementModel *)localElement toDestination:(SigMeshAddress *)destination usingApplicationKey:(SigAppkeyModel *)applicationKey command:(SDKLibCommand *)command;

/// Sends the Proxy Configuration Message to the connected Proxy Node.
/// @param message The Proxy Configuration message to be sent.
/// @param command The command save message and callback.
- (void)sendSigProxyConfigurationMessage:(SigProxyConfigurationMessage *)message command:(SDKLibCommand *)command;

/// Sends the telink's onlineStatus command.
/// @param message The onlineStatus message to be sent.
/// @param command The command save message and callback.
/// @returns return `nil` when send message successful.
- (NSError *)sendTelinkApiGetOnlineStatueFromUUIDWithMessage:(SigMeshMessage *)message command:(SDKLibCommand *)command;

/// Cancels sending the message with the given identifier.
/// @param messageId The message identifier.
- (void)cancelSigMessageHandle:(SigMessageHandle *)messageId;

/// cancel all commands and retry of commands and retry of segment PDU.
- (void)cleanAllCommandsAndRetry;

/// Returns whether SigMeshLib is busy. YES means busy.
- (BOOL)isBusyNow;

@end

NS_ASSUME_NONNULL_END
