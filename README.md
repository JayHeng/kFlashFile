# kFlashFile

## 一、简介

kFlashFile 是一个基于 NOR Flash 的轻量级文件数据存储方案，用于需要断电数据保存的项目。

kFlashFile 主要为 i.MXRT 系列设计，但其分层框架设计使其也可轻松移植到其他 MCU 平台。

kFlashFile 从设计上分为三层：

> * 最底层是Driver层：即Low-level驱动，这层是MCU相关的，对于i.MXRT来说，就是FlexSPI模块的驱动。
> * 中间是Adapter层：主要用于适配底层Driver，不同MCU其Driver接口函数可能不同，因此会在这一层做到接口统一。
> * 最顶层是API层：纯软件逻辑设计来实现文件数据存储，提供了四个非常简易的API。

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Framework.PNG)

## 二、设计

### 2.1 API定义

kFlashFile 是一个文件数据存储的设计，file_read()、file_save()是两个必备的 API，此外也提供业界通用 API 接口file_init()、file_deinit()。

> * kflash_file_init(): 用于初次分配Flash空间来存储文件数据，并且指定文件长度。如果当前指定的Flash空间里存在有效文件数据，那么继续复用。
> * kflash_file_read(): 用于获取当前有效存储的文件数据，文件数据可以部分读取。
> * kflash_file_save(): 用于实时写入最新的文件数据，文件数据可以部分更新。
> * kflash_file_deinit(): 用于清除当前分配的Flash空间里的文件数据，以便下次重新分配。

```C
status_t kflash_file_init(kflash_file_t *flashFile, uint32_t memStart, uint32_t memSize, uint32_t fileSize);
status_t kflash_file_read(kflash_file_t *flashFile, uint32_t offset, uint8_t *data, uint32_t size);
status_t kflash_file_save(kflash_file_t *flashFile, uint32_t offset, uint8_t *data, uint32_t size);
status_t kflash_file_deinit(kflash_file_t *flashFile);
```

### 2.2 空间分配

kFlashFile 将分配的 Flash 空间分成两个部分，前面是文件数据区（Data Sectors），后面是文件头区（Header Sectors）。

文件数据区：从区内起始地址开始按序存放一份份文件数据，只要文件数据出现无法覆盖的更新（即 Flash 无法改写的特性），便会在下一个新地址重新存储。如果数据区满了，便擦除区内起始地址处的历史文件数据，继续循环存储。

文件头区：区内 Sector 起始地址放一个 Magic 值（4字节），用于标识文件头。然后开始按序记录一份份文件数据在文件数据区里的位置信息（默认用 2byte 去记录一份文件数据的位置）。如果当前 Header Sector 存储满了，便换到下一个 Header Sector 继续记录。

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Design0.PNG)

### 2.3 API主参数

kFlashFile 设计上使用 kflash_file_t 型作为 API 主参数，这个参数原型定义如下：

```C
typedef struct {
    uint32_t managedStart;
    uint32_t managedSize;
    uint32_t activedStart;
    uint32_t activedSize;
    uint32_t recordedIdx;
    uint32_t recordedPos;
    uint8_t buffer[KFLASH_MAX_FILE_SIZE];
} kflash_file_t;
```

> * managedStart： 表示文件存储区映射首地址，即 kflash_file_init() 调用时的 memStart 值加上 Flash 在内存里映射首地址，managedStart 需要以 Flash Sector 大小对齐。
> * managedSize： 表示文件存储区总大小，即 kflash_file_init() 调用时的 memSize 值，需要是 Flash Sector 大小的整数倍。
> * activedStart： 表示当前有效文件数据存储的映射首地址，需要以 Flash Page 大小对齐。
> * activedSize： 表示当前有效文件数据长度，需要是 Flash Page 大小的整数倍。
> * recordedIdx： 表示当前有效文件头所在的 Header Sector 索引。
> * recordedPos： 表示 Header Sector 中用于存储当前有效文件数据位置信息的区域偏移。
> * buffer[]： 当前有效的文件数据暂存区。

## 三、实现

### 3.1 Driver层

在 i.MXRT 系列上，kFlashFile 的 Driver 层即 FlexSPI NOR 驱动，这个驱动既可以采用 MCU SDK 版本，也可以采用 BootROM 版本。

此处推荐 BootROM 版本的 FlexSPI NOR 驱动，因为这个驱动历经多个 MCU ROM 的洗礼，已经相当成熟稳定。这里简单讲下其中 Flash 操作的函数：

> * flexspi_nor_flash_erase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length)：这个函数实现Flash擦除，虽然形参里是任意设定的start, address，但实际擦除还是以Sector对齐的，函数内部会对start和address做自动对齐。
> * flexspi_nor_flash_page_program(uint32_t instance, flexspi_nor_config_t *config, uint32_t dstAddr, const uint32_t *src)：这个函数实现Flash编程，一次固定写一整个Page大小的数据，即使dstAddr不是以Page对齐，实际写入的Page数据也不会跨物理Page（会自动跳回同一个物理Page首地址，这是Flash自身特性）。

因为 flexspi_nor_flash_page_program() 每次都要固定编程整个 Page 数据，不够灵活，因此我新写了一个 flexspi_nor_flash_program() 函数，这个函数支持编程用户自定义长度的数据，并且支持跨物理 Page 去写：

> * flexspi_nor_flash_program(uint32_t instance, flexspi_nor_config_t *config, uint32_t dstAddr, const uint32_t *src, uint32_t length)：

需要特别注意，对于 SDR 模式的 Flash，最小编程长度可以是 1Byte；而 DDR 模式的 Flash，最小编程长度应是 2Bytes（如果这 2Bytes 地址上有一个 Byte 内容是 0xFF，该 Byte 依旧可以被再次编程）。

此外 flexspi_nor_flash_program() 函数有一个限制，即传入的 src 源数据首地址必须 4 字节对齐，哪怕你只想写入 2 个字节，这是 FlexSPI 模块底层对驱动的要求。

### 3.2 Adapter层

kFlashFile 的 Adapter 层是对 Driver 层做了一层封装，用于屏蔽硬件相关特性。该层与 MCU 以及板载 Flash 型号息息相关。下面的宏定义适用 i.MXRT1170 芯片以及连接在 FlexSPI1 上的 Octal Flash（MX25UM51345）：

```C
// 表示 Flash 连接的是 FlexSPI1
#define KFLASH_INSTANCE          (1)
// BootROM FlexSPI 驱动对 Octal Flash 支持的简易配置值
#define KFLASH_CONFIG_OPTION     (0xc0403007)

// FlexSPI1 在系统内存中的映射首地址
#define KFLASH_BASE_ADDRESS      (0x30000000)
// 默认的 Flash Sector/Page 大小（如果 Flash 里有 SFDP，则此处定义无效）
#define KFLASH_SECTOR_SIZE       (0x1000)
#define KFLASH_PAGE_SIZE         (256)

// FlexSPI 编程接口对传入的 src 源数据首地址必须 4 字节对齐
#define KFLASH_PROGRAM_ALIGNMENT (4)
// Flash SDR 模式为 1，DDR 模式为 2
#define KFLASH_PROGRAM_UNIT      (2)
```

kFlashFile 的 Adapter 层接口函数如下，参数是硬件无关的，因此上层可以轻松基于这些接口函数做纯软件逻辑设计。

```C
status_t kflash_drv_init(void);
uint32_t kflash_drv_get_info(kflash_mem_info_t flashInfo);
status_t kflash_drv_erase_region(uint32_t start, uint32_t length);
status_t kflash_drv_program_region(uint32_t dstAddr, const uint32_t *src, uint32_t length);
```

### 3.3 API层

kFlashFile 的 API 功能设计思路前面介绍过了，这里介绍具体代码实现，先来看几个关键的宏定义：

```C
// 设置 Header Sector 的个数，至少是 2 个
#define KFLASH_HDR_SECTORS     (2)
// 设置 Header Sector 中用于存储当前有效文件数据位置信息的区域存储类型
// uint16_t 最多可记录 65536 个位置，最大可支持的 Data 区域大小为 65536 * 文件数据长度
#define KFLASH_HDR_POS_TYPE    uint16_t   /* uint16_t or uint32_t */
// 设置总分配的 Flash 长度（Data+Header Sector 的个数），至少是 4 个
#define KFLASH_MIN_SECTORS     (KFLASH_HDR_SECTORS + 2)
// 设置最大支持的文件数据长度，需是 Flash Page 的整数倍
#define KFLASH_MAX_FILE_SIZE   (KFLASH_PAGE_SIZE * 2)
```

#### 3.3.1 init()

kflash_file_init() 函数处理流程如下：

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Flow_init.PNG)

如果是首次指定 Flash 空间，那么直接将全部空间擦除干净，并在第一个 Header Sector 中写入初始文件头（Magic + 文件数据位置值 0），即最新有效文件数据在 Flash 空间文件数据区的首地址。

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Design_Init.PNG)

这里有一个特殊的设计，文件数据区其实并不是直接存储用户写入的文件数据，而是将用户文件数据全部按位取反之后再存储进 Flash。这里假定用户数据初始应该是全 0，然后更改主要是将 0 值改为其他值，取反之后，正好对应 Flash 里的 bit1 编程为 bit0（Flash 擦除后是全 0xFF），这样可以充分利用 Flash 覆盖操作以减少擦除次数。

函数中比较关键的步骤是找寻当前 Flash 空间中是否存在有效文件数据，方法是遍历 Header Sector，发现存在 Magic 便继续寻找最新文件数据位置信息存放的区域（默认 2 字节），按照前面的设计，只需要按序读取区域内容，直到遇到 0xFFFF 为止。

#### 3.3.2 read()

kflash_file_read() 函数最简单了，直接从缓存区 buffer 里获取数据即可，因为每次更新文件数据操作完成之后都会将最新文件数据放在 buffer 里。

#### 3.3.3 save()

kflash_file_save() 函数是最核心的函数了，这里逻辑比较复杂，涉及文件数据区全部满了之后的动作，以及文件头区某个 Sector 满了的动作。其处理流程如下：

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Flow_save.PNG)

当有一个新文件数据要求保存时，首先会判断这个文件能不能在 Flash 中直接覆盖存储，如果能，那就直接覆盖存储，文件头完全不需要更新，这种情况比较简单。

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Design_Save0.PNG)

如果新文件数据无法直接覆盖存储，那么首先判断文件数据区是否满了，如果上一个文件数据已经存在了文件数据区的最后位置，此时需要擦除数据区第一个 Sector 从头开始存储。如果没有到最后位置，那就按序往下存储。

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Design_Save1.PNG)

新文件数据已经保存到数据区之后，此时需要处理文件头，记录这个新文件数据的位置。如果文件头区已经记录到当前 Sector 的最后位置，需要切换到下一个 Sector 开始存储，切换存储完新位置后，将之前 Sector 擦除。如果没有，那就按序在当前 Sector 继续记录。

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Design_Save2.PNG)

#### 3.3.4 deinit()

kflash_file_deinit() 函数也比较简单，就是将文件头区域 Header Sectors 全部擦除即可，文件数据区内容可以不用管，下次重新分配 Flash 时会做擦除。

![](https://raw.githubusercontent.com/JayHeng/kFlashFile/master/doc/kFlashFile_Design_Deinit.PNG)
