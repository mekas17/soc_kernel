/*
 *  T19XX dma definitions
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */



#ifndef DMA_H_
#define DMA_H_

/* DMA request source num*/
#if 0
#define DMA_RQ_UART0		0
#define DMA_RQ_UART1		1
#define DMA_RQ_AUDIORX		2
#define DMA_RQ_AUDIOTX		3
#define DMA_RQ_IISRX		4
#define DMA_RQ_IISTX		5
#define DMA_RQ_UART2 	6
#define DMA_RQ_SMCARD	7

#define DMA_RQ_SDMMCRX	8
#define DMA_RQ_SDIO		9
#define DMA_RQ_SDMMCTX	8

#define DMA_RQ_SPIRX		14
#define DMA_RQ_SPITX		15

#else

#define DMA_RQ_I2S0_RX0                       0 // *
#define DMA_RQ_I2S0_TX0                       1 // *

#define DMA_RQ_SDMMCRX                        2
#define DMA_RQ_SDMMCTX                        2

//#define DMA_RQ_SDIO                           3
#define DMA_RQ_SDIORX						3
#define DMA_RQ_SDIOTX						3

#define DMA_RQ_SPIRX                        4
#define DMA_RQ_SPITX                        5

#define DMA_RQ_I2S1_RX0                       6 // *
#define DMA_RQ_I2S1_TX0                       7 // *
#define DMA_RQ_I2S1_RX1                       8 // *
#define DMA_RQ_I2S1_TX1                       9 // *

#define DMA_RQ_I2S2_RX0                       10 // *
#define DMA_RQ_I2S2_TX0                       11 // *
#define DMA_RQ_I2S2_RX1                       12 // *
#define DMA_RQ_I2S2_TX1                       13 // *

#define DMA_RQ_UART6_TX                       14 // *
#define DMA_RQ_UART6_RX                       15 // *

#define DMA_RQ_UART5_TX                       16 // *
#define DMA_RQ_UART5_RX                       17 // *

#define DMA_RQ_UART1_TX                       18 // *
#define DMA_RQ_UART1_RX                       19 // *

#define DMA_RQ_UART2_TX                       20 // *
#define DMA_RQ_UART2_RX                       21 // *

#define DMA_RQ_I2C_TX                         22
#define DMA_RQ_I2C_RX                         23

#define DMA_RQ_UART3_TX                       24 // *
#define DMA_RQ_UART3_RX                       25 // *

#define DMA_RQ_I2C1_TX                        26
#define DMA_RQ_I2C1_RX                        27

#define DMA_RQ_UART4_TX                       28 // *
#define DMA_RQ_UART4_RX                       29 // *

#define DMA_RQ_SPI1_RX                        30
#define DMA_RQ_SPI1_TX                        31

#endif


#endif

