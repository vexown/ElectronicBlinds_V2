/*
 * lwipopts.h - Configuration Options for lwIP (Lightweight IP)
 *
 * Purpose:
 *   This file is used to configure the behavior of the lwIP stack for the specific
 *   needs of the embedded project. It allows customization of various features such
 *   as memory management, network protocols, debugging, and integration with an OS.
 *
 * Configuration:
 *   - NO_SYS: Set to 0 to enable OS support (e.g., FreeRTOS). Set to 1 for a bare-metal
 *             environment without OS support.
 *   - MEM_SIZE: Total amount of memory available for lwIP’s heap.
 *   - MEMP_NUM_*: Number of memory pools for different object types (e.g., pbufs, TCP PCBs).
 *   - TCP_MSS: Maximum segment size for TCP connections.
 *   - TCP_SND_BUF, TCP_WND: Buffer sizes for TCP send and receive windows.
 *   - LWIP_UDP: Set to 1 to enable UDP support.
 *   - IP_REASSEMBLY: Enable or disable IP packet reassembly.
 *   - LWIP_DEBUG: Enable or disable debugging features and logging.
 *
 * Notes:
 *   - Modify these settings based on the specific requirements of your application,
 *     including memory constraints, performance needs, and debugging requirements.
 *   - Ensure that this file is correctly included in the build configuration and that
 *     paths are set appropriately.
 *
 */

#ifndef _LWIPOPTS_EXAMPLE_COMMONH_H
#define _LWIPOPTS_EXAMPLE_COMMONH_H


/* Common settings used in most of the pico_w examples.
 * For detailed explanations, refer to the official lwIP documentation:
 * https://www.nongnu.org/lwip/2_1_x/group__lwip__opts.html
 */

/* Allow override in some examples. Determines if the lwIP stack runs without an operating system (1) 
 * or with an OS (0). */
#ifndef NO_SYS
#define NO_SYS                      0
#endif

/* Allow override in some examples. Enables lwIP socket API (1). */
#ifndef LWIP_SOCKET
#define LWIP_SOCKET                 1
#endif

#if PICO_CYW43_ARCH_POLL
/* Use system's malloc for memory allocation if polling is enabled. */
#define MEM_LIBC_MALLOC             1
#else
/* Disable system's malloc for non-polling configurations. */
#define MEM_LIBC_MALLOC             0
#endif

/* Memory alignment for the lwIP stack. Usually set to match the system's architecture. */
#define MEM_ALIGNMENT               4

/* Size of the heap for memory allocation (in bytes). Adjust based on application needs.
   This value determines the total amount of memory available for lwIP’s dynamic allocations. */
#define MEM_SIZE                    (32 * 1024) // 32KB heap

/* Number of TCP segments that can be queued. Increase for more simultaneous TCP connections. 
   This value is calculated based on the size of the TCP send buffer (TCP_SND_BUF) and the maximum segment size (TCP_MSS) 
   The factor of 4 ensures there are enough segments to handle retransmissions and window scaling. */
#define MEMP_NUM_TCP_SEG           ((4 * TCP_SND_BUF) / TCP_MSS)

/* Number of entries in the ARP queue. Increase for high ARP traffic environments. */
#define MEMP_NUM_ARP_QUEUE          10

/* Number of buffers in the packet buffer pool. Adjust based on expected network traffic. 
   A higher number allows more simultaneous packets to be processed but uses more memory. */
#define PBUF_POOL_SIZE              24
/* Size of each buffer (pbuf) in the packet buffer pool. 
   This ensures most packets can be stored in a single pbuf without fragmentation */
#define PBUF_POOL_BUFSIZE           1536 //  1536 bytes is chosen to accommodate a full Ethernet frame (1500 bytes payload + headers).

/* Enable ARP functionality (1). Required for Ethernet and IPv4. */
#define LWIP_ARP                    1

/* Enable Ethernet support (1). */
#define LWIP_ETHERNET               1

/* Enable ICMP (Internet Control Message Protocol) support (1). */
#define LWIP_ICMP                   1

/* Enable raw API support (1) for custom low-level protocol handling. */
#define LWIP_RAW                    1

/* Enable window scaling */
#define LWIP_WND_SCALE              0

/* If window scaling is enabled, set the scale factor. */
#if (LWIP_WND_SCALE == 1)
/* Use a moderate scale factor (2^TCP_RCV_SCALE = 2^2 = 4x multiplier) */
#define TCP_RCV_SCALE               2
#endif

/* Maximum segment size for TCP.
   1460 bytes is chosen to fit within the standard Ethernet MTU (1500 bytes) minus the IP (20 bytes) and TCP (20 bytes) headers. */
#define TCP_MSS                     1460 

/* TCP Receive Window Size in bytes (Increase for better throughput) */
#define TCP_WND                     (32 * 1024)  // 32KB base

/* Size of the TCP send buffer in bytes. Increase for larger TCP transmissions. */
#define TCP_SND_BUF                 (TCP_WND/2) // Half of the receive window is a good default

/* Threshold for updating the TCP window size. */
#define TCP_WND_UPDATE_THRESHOLD    (TCP_WND/4)

/* Length of the TCP send queue. Determines how many segments can be queued for sending. */
#define TCP_SND_QUEUELEN           ((4 * TCP_SND_BUF) / TCP_MSS)

/* Enable callback for network interface status changes (1). */
#define LWIP_NETIF_STATUS_CALLBACK  1

/* Enable callback for link status changes on network interfaces (1). */
#define LWIP_NETIF_LINK_CALLBACK    1

/* Enable hostname support for network interfaces (1). */
#define LWIP_NETIF_HOSTNAME         1

/* Disable Netconn API (0) as it's not used in this configuration. */
#define LWIP_NETCONN                0

/* Disable memory usage statistics (0). */
#define MEM_STATS                   0

/* Disable system statistics (0). */
#define SYS_STATS                   0

/* Disable memory pool statistics (0). */
#define MEMP_STATS                  0

/* Disable link statistics (0). */
#define LINK_STATS                  0

/* Padding added to Ethernet frames. Uncomment and set as needed. */
// #define ETH_PAD_SIZE                2

/* Select checksum algorithm (3 = fastest option on most systems). */
#define LWIP_CHKSUM_ALGORITHM       3

/* Enable DHCP (Dynamic Host Configuration Protocol) support (1). */
#define LWIP_DHCP                   1

/* Enable IPv4 support (1). */
#define LWIP_IPV4                   1

/* Enable TCP support (1). */
#define LWIP_TCP                    1

/* Enable UDP support (1). */
#define LWIP_UDP                    1

/* Enable DNS (Domain Name System) support (1). */
#define LWIP_DNS                    1

/* Enable TCP keepalive mechanism (1). */
#define LWIP_TCP_KEEPALIVE          1

/* Optimize for single-buffer transmissions (1). */
#define LWIP_NETIF_TX_SINGLE_PBUF   1

/* Disable ARP check during DHCP address assignment (0). */
#define DHCP_DOES_ARP_CHECK         0

/* Disable ACD (Address Conflict Detection) checks for DHCP (0). */
#define LWIP_DHCP_DOES_ACD_CHECK    0

#ifndef NDEBUG
/* Enable debugging and statistics when not in release mode. */
#define LWIP_DEBUG                  1
#define LWIP_STATS                  1
#define LWIP_STATS_DISPLAY          1
#endif

/* Debug settings: disable debugging for various lwIP components. */
#define ETHARP_DEBUG                LWIP_DBG_OFF
#define NETIF_DEBUG                 LWIP_DBG_OFF
#define PBUF_DEBUG                  LWIP_DBG_OFF
#define API_LIB_DEBUG               LWIP_DBG_OFF
#define API_MSG_DEBUG               LWIP_DBG_OFF
#define SOCKETS_DEBUG               LWIP_DBG_OFF
#define ICMP_DEBUG                  LWIP_DBG_OFF
#define INET_DEBUG                  LWIP_DBG_OFF
#define IP_DEBUG                    LWIP_DBG_OFF
#define IP_REASS_DEBUG              LWIP_DBG_OFF
#define RAW_DEBUG                   LWIP_DBG_OFF
#define MEM_DEBUG                   LWIP_DBG_OFF
#define MEMP_DEBUG                  LWIP_DBG_OFF
#define SYS_DEBUG                   LWIP_DBG_OFF
#define TCP_DEBUG                   LWIP_DBG_OFF
#define TCP_INPUT_DEBUG             LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG            LWIP_DBG_OFF
#define TCP_RTO_DEBUG               LWIP_DBG_OFF
#define TCP_CWND_DEBUG              LWIP_DBG_OFF
#define TCP_WND_DEBUG               LWIP_DBG_OFF
#define TCP_FR_DEBUG                LWIP_DBG_OFF
#define TCP_QLEN_DEBUG              LWIP_DBG_OFF
#define TCP_RST_DEBUG               LWIP_DBG_OFF
#define UDP_DEBUG                   LWIP_DBG_OFF
#define TCPIP_DEBUG                 LWIP_DBG_OFF
#define PPP_DEBUG                   LWIP_DBG_OFF
#define SLIP_DEBUG                  LWIP_DBG_OFF
#define DHCP_DEBUG                  LWIP_DBG_OFF

#if !NO_SYS
/* Stack size for the tcpip_thread task. */
#define TCPIP_THREAD_STACKSIZE 2048

/* Default stack size for other lwIP threads. */
#define DEFAULT_THREAD_STACKSIZE 1024

/* Mailbox size for raw protocol receive operations. */
#define DEFAULT_RAW_RECVMBOX_SIZE 8

/* Mailbox size for UDP receive operations. */
#define DEFAULT_UDP_RECVMBOX_SIZE 8

/* Mailbox size for TCP receive operations. */
#define DEFAULT_TCP_RECVMBOX_SIZE 8

/* Mailbox size for the tcpip_thread task. */
#define TCPIP_MBOX_SIZE 8

/* Use system-provided timeval structure (0). */
#define LWIP_TIMEVAL_PRIVATE 0

/* Enable core locking for tcpip_thread input. */
#define LWIP_TCPIP_CORE_LOCKING_INPUT 1

/* Enable socket receive timeout feature. */
#define LWIP_SO_RCVTIMEO 1
#endif

#endif /* __LWIPOPTS_H__ */
