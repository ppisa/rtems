#include <bsp/dma.h>
#include <bsp/vc.h>
#include <libcpu/arm-cp15.h>
#include <bsp.h>

volatile int continue_execution;

/*        CRn opc1 CRm opc2
ATS12NSOPW c7 0 c8 5 32-bit WO Stages 1 and 2 Non-secure only PL1 write
ATS1CPW    c7 0 c8 1 32-bit WO Stage 1 Current state PL1 write
*/
/*
MCR p15, <opc1>, <Rt>, c7, c8, <opc2> ; Address translation operation, as defined by <opc1> and <opc2>
MRC p15, 0, <Rt>, c7, c4, 0 ; Read 32-bit PAR into Rt
MRRC p15, 0, <Rt>, <Rt2>, c7 ; Read 64-bit PAR into Rt (low word) and Rt2 (high word)
*/

ARM_CP15_TEXT_SECTION static inline uint32_t
arm_cp15_ATS1CPW(uint32_t virt_addr)
{
  ARM_SWITCH_REGISTERS;
  uint32_t phys_addr;

  __asm__ volatile (
    ARM_SWITCH_TO_ARM
    "mcr p15, 0, %[va], c7, c8, 1\n"
   #if defined(__ARM_ARCH_7A__)
    "isb\n"
   #endif
    "mrc p15, 0, %[pa], c7, c4, 0\n"
    ARM_SWITCH_BACK
    : [pa] "=&r" (phys_addr) ARM_SWITCH_ADDITIONAL_OUTPUT
    : [va] "r" (virt_addr)
  );

  return phys_addr;
}

void print_translations_for_range(uint32_t va_start, uint32_t va_end, uint32_t step)
{
  uint32_t pa;
  uint32_t va;

  for ( va = va_start; va < va_end; va += step) {
    pa = arm_cp15_ATS1CPW(va);
    printf("0x%08lx -> 0x%08lx LPAE %lu NOS %lu NS %lu ?? %lu SH %lu Inner %lu Outer %lu SS %lu F %lu\n",
           va, pa, (pa >> 11) & 1, (pa >> 10) & 1, (pa >> 9) & 1, (pa >> 8) & 1, (pa >> 7) & 1,
           (pa >> 4) & 7, (pa >> 2) & 3, (pa >> 1) & 1, (pa >> 0) & 1);
  }
}

int experiment_print_translations(int argc, char *argv[])
{
  uint32_t mb = 1024 * 1024;

  print_translations_for_range(0, 10 * mb, mb);

  return 0;
}

/* VideoCore initialization experiment based on code written by Goswin von Brederlow */

/* https://github.com/mrvn/test/blob/master/framebuffer.cc */
/* https://github.com/raspberrypi/firmware/wiki/Mailbox-property-interface */

enum Error {
  SUCCESS,
  // Error codes
  FAIL_GET_RESOLUTION,
  FAIL_GOT_INVALID_RESOLUTION,
  FAIL_SETUP_FRAMEBUFFER,
  FAIL_INVALID_TAGS,
  FAIL_INVALID_TAG_RESPONSE,
  FAIL_INVALID_TAG_DATA,
  FAIL_INVALID_PITCH_RESPONSE,
  FAIL_INVALID_PITCH_DATA
};

uint32_t mailbuffer[1024] __attribute__((aligned(64)));

typedef struct {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  uint8_t alpha;
} Pixel;

typedef struct {
  uint32_t width;
  uint32_t height;
  uint32_t pitch;
  uint32_t base;
  uint32_t size;
} vc_fb_t;

vc_fb_t fb;

#if ( RPI_L2_CACHE_ENABLE == 1 )
  #define BCM2835_VC_MEMORY_MAPPING 0x40000000
#else
  #define BCM2835_VC_MEMORY_MAPPING 0xC0000000
#endif

uint32_t *vc_send_and_read_buffer(void *buf, size_t size)
{
  rtems_cache_flush_multiple_data_lines( buf, size );
  RTEMS_COMPILER_MEMORY_BARRIER();
  rtems_cache_invalidate_multiple_data_lines( buf, size );

  RTEMS_COMPILER_MEMORY_BARRIER();
  raspberrypi_mailbox_write( BCM2835_MBOX_CHANNEL_PROP_AVC,
    (unsigned int) buf + BCM2835_VC_MEMORY_MAPPING );
  RTEMS_COMPILER_MEMORY_BARRIER();
  raspberrypi_mailbox_read( BCM2835_MBOX_CHANNEL_PROP_AVC );
  RTEMS_COMPILER_MEMORY_BARRIER();
  return buf;
}

int vc_init(void)
{
  uint32_t *buf;

  puts("Framebuffer vc_init()\n");

  //fb.width = 1920;
  //fb.height = 1080;

  // some spare memory for mail content

  /* Get the display size */
  mailbuffer[0] = 8 * 4; // Total size
  mailbuffer[1] = 0; // Request
  mailbuffer[2] = 0x40003; // Display size
  mailbuffer[3] = 8; // Buffer size
  mailbuffer[4] = 0; // Request size
  mailbuffer[5] = 0; // Space for horizontal resolution
  mailbuffer[6] = 0; // Space for vertical resolution
  mailbuffer[7] = 0; // End tag

  buf = vc_send_and_read_buffer(mailbuffer, 8 * 4);

  /* Valid response in data structure? */
  if (buf[1] != 0x80000000) return FAIL_GET_RESOLUTION;

  fb.width = buf[5];
  fb.height = buf[6];

  if (fb.width == 0 || fb.height == 0) return FAIL_GOT_INVALID_RESOLUTION;

  /* Set up screen */
  unsigned int c = 1;
  mailbuffer[c++] = 0; // Request

  mailbuffer[c++] = 0x00048003; // Tag id (set physical size)
  mailbuffer[c++] = 8; // Value buffer size (bytes)
  mailbuffer[c++] = 8; // Req. + value length (bytes)
  mailbuffer[c++] = fb.width; // Horizontal resolution
  mailbuffer[c++] = fb.height; // Vertical resolution

  mailbuffer[c++] = 0x00048004; // Tag id (set virtual size)
  mailbuffer[c++] = 8; // Value buffer size (bytes)
  mailbuffer[c++] = 8; // Req. + value length (bytes)
  mailbuffer[c++] = fb.width; // Horizontal resolution
  mailbuffer[c++] = fb.height; // Vertical resolution

  mailbuffer[c++] = 0x00048005; // Tag id (set depth)
  mailbuffer[c++] = 4; // Value buffer size (bytes)
  mailbuffer[c++] = 4; // Req. + value length (bytes)
  mailbuffer[c++] = 32; // 32 bpp

  mailbuffer[c++] = 0x00040001; // Tag id (allocate framebuffer)
  mailbuffer[c++] = 8; // Value buffer size (bytes)
  mailbuffer[c++] = 4; // Req. + value length (bytes)
  mailbuffer[c++] = 16; // Alignment = 16
  mailbuffer[c++] = 0; // Space for response

  mailbuffer[c++] = 0; // Terminating tag

  mailbuffer[0] = c*4; // Buffer size

  buf = vc_send_and_read_buffer(mailbuffer, c*4);

  /* Valid response in data structure */
  if(buf[1] != 0x80000000) return FAIL_SETUP_FRAMEBUFFER;

  // Scan replies for allocate response
  unsigned int i = 2; /* First tag */
  uint32_t data;
  while ((data = buf[i])) {
      // allocate response?
      if (data == 0x40001) break;

      /* Skip to next tag
       * Advance count by 1 (tag) + 2 (buffer size/value size)
       * + specified buffer size
       */
      i += 3 + (buf[i + 1] >> 2);

      if (i > c) return FAIL_INVALID_TAGS;
  }

  /* 8 bytes, plus MSB set to indicate a response */
  if (buf[i + 2] != 0x80000008) return FAIL_INVALID_TAG_RESPONSE;

  /* Framebuffer address/size in response */
  fb.base = buf[i + 3];
  fb.size = buf[i + 4];

  if (fb.base == 0 || fb.size == 0) return FAIL_INVALID_TAG_DATA;

  // fb.base += 0xC0000000; // physical to virtual

  /* Get the framebuffer pitch (bytes per line) */
  mailbuffer[0] = 7 * 4; // Total size
  mailbuffer[1] = 0; // Request
  mailbuffer[2] = 0x40008; // Display size
  mailbuffer[3] = 4; // Buffer size
  mailbuffer[4] = 0; // Request size
  mailbuffer[5] = 0; // Space for pitch
  mailbuffer[6] = 0; // End tag

  buf = vc_send_and_read_buffer(mailbuffer, 7 * 4);

  /* 4 bytes, plus MSB set to indicate a response */
  if (buf[4] != 0x80000004) return FAIL_INVALID_PITCH_RESPONSE;

  fb.pitch = buf[5];
  if (fb.pitch == 0) return FAIL_INVALID_PITCH_DATA;

  printf("fb %p 0x%08x %u x %u\n", fb.base, fb.size, fb.width, fb.height);

  fb.base &= 0x3fffffff;

  unsigned int flg = ARMV7_MMU_DATA_READ_WRITE;
  //unsigned int flg = ARMV7_MMU_DATA_READ_WRITE_CACHED;
  //unsigned int flg = ARMV7_MMU_READ_WRITE | ARM_MMU_SECT_TEX_0 | ARM_MMU_SECT_C | ARM_MMU_SECT_B | ARM_MMU_SECT_S;

  void *p = (void *)fb.base;
  arm_cp15_set_translation_table_entries(p, p + fb.size, flg );

  print_translations_for_range(fb.base, fb.base + 1024 * 1024, 1024 * 1024);


  if (0) {
    ll_strout("JTAG Enabled and waiting for GDB\n");

    continue_execution = 0;
    while (!continue_execution);
  }

  // set alpha to 0xff everywhere
  for(uint32_t n = 3; n < fb.size; n += 4) {
      *(uint8_t*)(fb.base + n) = 0xff;
  }

  // draw chessboard pattern
  for(uint32_t y = 0; y < fb.height; ++y) {
    for(uint32_t x = 0; x < fb.width; ++x) {
      Pixel *p = (Pixel*)(fb.base + x * sizeof(Pixel) + y * fb.pitch);
      uint8_t col = ((x & 16) ^ (y & 16)) ? 0x00 : 0xff;
      p->red = col;
      p->green = col;
      p->blue = col;
    }
  }

  // draw back->red fade left to right at the top
  // draw back->blue fade left to right at the bottom
  for(int y = 0; y < 16; ++y) {
    for(int x = 16; x < 256 + 16; ++x) {
      Pixel *p = (Pixel*)(fb.base + x * sizeof(Pixel) + y * fb.pitch);
      p->red = x - 16;
      p->green = 0;
      p->blue = 0;
      p = (Pixel*)(fb.base + x * sizeof(Pixel) + (fb.height - y - 1) * fb.pitch);
      p->red = 0;
      p->green = 0;
      p->blue = x - 16;
    }
  }
  // draw back->green fade top to bottom at the left
  // draw back->green fade top to bottom at the right
  for(int y = 16; y < 256 + 16; ++y) {
    for(int x = 0; x < 16; ++x) {
      Pixel *p = (Pixel*)(fb.base + x * sizeof(Pixel) + y * fb.pitch);
      p->red = 0;
      p->green = y - 16;
      p->blue = 0;
      p = (Pixel*)(fb.base + (fb.width - x- 1) * sizeof(Pixel) + y * fb.pitch);
      p->red = y - 16;
      p->green = y - 16;
      p->blue = y - 16;
    }
  }

  //rtems_cache_flush_multiple_data_lines( (void*)fb.base, fb.size);

#if 0

  const char text[] = "MOOSE V0.0";
  struct Arg {
    uint32_t color;
    uint32_t border;
    bool fill;
  } args[] = {
    {~0LU,  0, false},
    { 0, ~0LU, false},
    {~0LU,  0, true},
    { 0, ~0LU, true},
    {0xff0000ff,  0,   false},
    {0xff0000ff, ~0LU, false},
    {0xff0000ff,  0,   true},
    {0xff0000ff, ~0LU, true},
    {0xff00ff00,  0,   false},
    {0xff00ff00, ~0LU, false},
    {0xff00ff00,  0,   true},
    {0xff00ff00, ~0LU, true},
    {0xffff0000,  0,   false},
    {0xffff0000, ~0LU, false},
    {0xffff0000,  0,   true},
    {0xffff0000, ~0LU, true},
  };
  int y = 152;
  Arg *arg = args;
  for (i = 0; i < 16; ++i) {
    int x = 152;
    for (const char *p = text; *p; ++p) {
      Font::putc(fb, x, y, *p, arg->color, arg->border, arg->fill);
      x += 8;
    }
    y += 16;
    ++arg;
  }
#endif
  return SUCCESS;
}

int experiment_vc_init(int argc, char *argv[])
{
  int res;
  res = vc_init();
  printf("vc_init returned %d\n", res);

  return 0;
}

void print_ver(void)
{
  int res;
  union {
    bcm2835_mailbox_get_fw_rev_entries fw_rev;
    bcm2835_get_board_spec_entries board_spec;
    bcm2835_get_board_spec_entries board_spec1;
  } u;
  printk("bcm2835_mailbox_get_firmware_revision ...\n");
  res = bcm2835_mailbox_get_firmware_revision(&u.fw_rev);
  printk("bcm2835_mailbox_get_firmware_revision %i %lu\n", res, u.fw_rev.fw_rev);

  printk("bcm2835_mailbox_get_board_model ...\n");
  res = bcm2835_mailbox_get_board_model(&u.board_spec);
  printk("bcm2835_mailbox_get_board_model %i %lu\n", res, u.board_spec.spec);

  printk("bcm2835_mailbox_get_board_revision ...\n");
  res = bcm2835_mailbox_get_board_revision(&u.board_spec1);
  printk("bcm2835_mailbox_get_board_revision %i %lu\n", res, u.board_spec1.spec);
}

int experiment_print_ver(int argc, char *argv[])
{
  int i;

  for ( i = 0 ; i < 10; i++ )
    print_ver();

  return 0;
}

typedef struct test_dma_state {

} test_dma_state_t;

#define TEST_DMA_BUFF_SIZE 1024

uint8_t test_dma_src[TEST_DMA_BUFF_SIZE] CPU_STRUCTURE_ALIGNMENT = {
 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0xaa, 0xbb, 0xcc
};

uint8_t test_dma_dst[TEST_DMA_BUFF_SIZE] CPU_STRUCTURE_ALIGNMENT;

test_dma_state_t test_dma_state;

void test_dma_intr(int dmach, void *context)
{
  printk("test_dma_intr %d %p\n", dmach, context);
}

int experiment_test_dma(int argc, char *argv[])
{
  int res;
  int dmach = 0;
  int i;
  int reqlen = 8;
  int cmperr = 0;

  printk("rpi_dma_init ...\n");
  res = rpi_dma_init(dmach);
  printk("rpi_dma_init %d\n", res);

  printk("rpi_dma_allocate ...\n");
  res = rpi_dma_allocate(dmach);
  printk("rpi_dma_allocate %d\n", res);

  printk("rpi_dma_setup_src ...\n");
  res = rpi_dma_setup_src(dmach, BCM_DMA_DREQ_NONE, BCM_DMA_INC_ADDR, BCM_DMA_32BIT);
  printk("rpi_dma_setup_src %d\n", res);

  printk("rpi_dma_setup_dst ...\n");
  res = rpi_dma_setup_dst(dmach, BCM_DMA_DREQ_NONE, BCM_DMA_INC_ADDR, BCM_DMA_32BIT);
  printk("rpi_dma_setup_dst %d\n", res);

  printk("rpi_dma_setup_intr ...\n");
  res = rpi_dma_setup_intr(dmach, test_dma_intr, &test_dma_state);
  printk("rpi_dma_setup_intr %d\n", res);

  rtems_cache_flush_multiple_data_lines( test_dma_src, reqlen );
  rtems_cache_invalidate_multiple_data_lines( test_dma_dst, reqlen );

  printk("rpi_dma_start ...\n");
  res = rpi_dma_start(dmach, test_dma_src, test_dma_dst, reqlen);
  printk("rpi_dma_start %d\n", res);

  rtems_task_wake_after(10);

  printf("dma test result\n");
  for ( i = 0; i < reqlen; i++) {
    printf("%02x>%02x ", test_dma_src[i], test_dma_dst[i]);
    if ( test_dma_src[i] != test_dma_dst[i] )
      cmperr = 1;
  }
  printf("\n");
  if ( cmperr )
    printf("simple mem2mem DMA test has FAILED\n");
  else
    printf("simple mem2mem DMA test has PASSED\n");

  return 0;
}

void experiments_run(void)
{
  experiment_print_translations(0, NULL);
  experiment_print_ver(0, NULL);
  //experiment_vc_init(0, NULL);
  experiment_test_dma(0, NULL);
}
