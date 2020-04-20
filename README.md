# uboot with Caisteal imx6ul support 

## Getting Started

### Prerequisites
The following packages are generically required for development work on u-boot.
It is strongly advised to check [uboot](https://github.com/u-boot/u-boot) repository for greater details
```
gawk wget git-core diffstat unzip build-essential chrpath libsdl1.2-dev xterm curl
texinfo lzop nfs-kernel-server gcc-arm-linux-gnueabihf bison flex
```

### Environment Variables
Set the following Environment Variables
```
export CROSS_COMPILE=arm-linux-gnueabihf-
export ARCH=arm
```

-----------------------------------------------------------------------------------------------------------------

## Cloning and Building Code

### Cloning Sourcecode

Activities are from home directory, select accordingly
```    
    cd <working dir>
    git clone https://github.com/summertanks/uboot-caisteal-2019.07.git
```

### Creating own repo
Incase you need to modify repo of own design
```
    cd uboot-caisteal-2019.07
    rm -rf .git
    git init
    git config --global user.email "name@example.com"
    git config --global user.name "name"
    git add .
```

Modify .gitignore to add /.pc/* after the last line
```
    git commit -m "Initial Version"
    git remote add origin https://github.com/<username>/<repo>.git
    git push -u origin master
    git branch zeus
    git push origin zeus
    git checkout zeus
```

### Build

Done from within the working directory (make sure the environment variables are set)
```
    make distclean
    make mx6caisteal_config
    make
```

------------------------------------------------------------------------------------------------------------------

## Internals

### Files Modified
```
    F arch/arm/mach-imx/mx6/Kconfig
    D board/caisteal
    D board/caisteal/mx6caisteal
    D board/causteal/common
    F board/caisteal/mx6caisteal/Kconfig
    F board/caisteal/mx6caisteal/README
    F board/caisteal/mx6caisteal/MAINTAINERS
    F board/caisteal/mx6caisteal/Makefile
    F board/caisteal/mx6caisteal/mx6caisteal.c
    F board/caisteal/common/Makefile
    F board/caisteal/common/pfuze.h
    F board/caisteal/common/pfuze.c
    F include/configs/mx6caisteal.h
    F arch/arm/dts/imx6ul-caisteal.dts
    F arch/arm/dts/Makefile
    F arch/arm/mach-imx/mx6/Kconfig
    F configs/mx6caisteal_defconfig
```


### Modified Sections

arch/arm/mach-imx/mx6/Kconfig
```
    config TARGET_MX6CAISTEAL
      	bool "mx6caisteal"
        select BOARD_LATE_INIT
        select DM
        select DM_THERMAL
        select MX6UL
        select SUPPORT_SPL
	imply CMD_DM

        source "board/caisteal/mx6caisteal/Kconfig"
```

Create new board
    Searching for files to modify - grep -ri "TARGET_MX6"

    # Modify arch/arm/mach-imx/mx6/Kconfig and add a section and nake of Kconfig
	config TARGET_MX6CAISTEAL
        	bool "mx6caisteal"
        	select BOARD_LATE_INIT
        	select DM
        	select DM_THERMAL
        	select MX6UL
       		select SUPPORT_SPL
        	imply CMD_DM

    	source "board/caisteal/mx6caisteal/Kconfig"

    # Creating the file will require to create the directories too
    	mkdir board/caisteal
    	mkdir board/caisteal/mx6caisteal
    	cp board/freescale/mx6ul_14x14_evk/Kconfig board/caisteal/mx6caisteal/Kconfig
    
    # Modify kconfig file with only the following content
    	if TARGET_MX6CAISTEAL

    	config SYS_BOARD
        	default "mx6caisteal"

    	config SYS_VENDOR
        	default "caisteal"

    	config SYS_CONFIG_NAME
        	default "mx6caisteal"

    	endif

    current files in board/freescale/mx6sabresd/ that we should be replicated
    -rw-r--r--  1 summertanks summertanks   154 Jul 12 21:44 Kconfig
    -rw-r--r--  1 summertanks summertanks   167 Jul 12 21:44 MAINTAINERS
    -rw-r--r--  1 summertanks summertanks   172 Jul 12 21:44 Makefile
    -rw-r--r--  1 summertanks summertanks 28679 Jul 12 21:44 mx6sabresd.c
    -rw-r--r--  1 summertanks summertanks  2512 Jul 12 21:44 README

    # Kconfig is already done

    # MAINTAINERS
    	cp board/freescale/mx6sabresd/MAINTAINERS board/caisteal/mx6caisteal/
    
    # Modify MAINTAINERS 
        MX6CAISTEAL BOARD
        M:      Harkirat S Virk <harkiratsvirk@gmail.com>
        S:      Maintained
        F:      board/caisteal/mx6caisteal/
        F:      include/configs/mx6caisteal.h   # additional file to duplicate
        F:      configs/mx6caisteal_defconfig   # additional file to duplicate

    # Makefile
    	cp board/freescale/mx6ul_14x14_evk/Makefile board/caisteal/mx6caisteal/

    # Modify file
        obj-y  := mx6caisteal.o

    # mx6caisteal.h
    	cp include/configs/mx6ul_14x14_evk.h include/configs/mx6caisteal.h
    # Modify lines    
        #ifndef __MX6CAISTEAL_CONFIG_H
        #define __MX6CAISTEAL_CONFIG_H

    # TODO: Check if this is still required
	#define is_mx6ul_9x9_evk()      CONFIG_IS_ENABLED(TARGET_MX6UL_9X9_EVK)
    
    # Modify
                "findfdt="\
	                "if test $fdt_file = undefined; then " \
        	                "if test $board_name = CAISTEAL && test $board_rev = 14X14; then " \
                                        "setenv fdt_file imx6ul-caisteal.dtb; fi; " \
                                "if test $fdt_file = undefined; then " \
                                        "echo WARNING: Could not determine dtb to use; fi; " \
                        "fi;\0" \

    #   imx6ul-caisteal.dts
	cp arch/arm/dts/imx6ul-14x14-evk.dts arch/arm/dts/imx6ul-caisteal.dts

    # Modify
        model = "Caisteal i.MX6 UltraLite 14x14";
        compatible = "fsl,imx6ul-14x14-evk", "fsl,imx6ul";

    #   mx6caisteal_defconfig
	cp configs/mx6ul_14x14_evk_defconfig configs/mx6caisteal_defconfig

    # Modify Lines
        CONFIG_TARGET_MX6CAISTEAL=y
        CONFIG_DEFAULT_DEVICE_TREE="imx6ul-14x14-evk"   # TODO : not changed yet


    # mx6caisteal.c
	cp board/freescale/mx6ul_14x14_evk/mx6ul_14x14_evk.c board/caisteal/mx6caisteal/mx6caisteal.c

    # Modify
	#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
        	env_set("board_name", "CAISTEAL");
        	env_set("board_rev", "14X14");
	#endif

	int checkboard(void)
	    {
        	puts("Board: Caisteal MX6UL 14x14 \n");
        	return 0;
	    }

    # pfuze.c/pfuze.h - required by mx6caisteal.c
	mkdir board/caisteal/common
	cp board/freescale/common/pfuze.* board/caisteal/common
	cp board/freescale/common/Makefile board/caisteal/common

    # Modify pfuze.c
    # modift pfuse.h
    # Modify Makefile
        comment out all except obj-$(CONFIG_DM_PMIC_PFUZE100) += pfuze.o 

DDR3 Calibration 
    (DDR3 Stress tester)

----------------------------------------------------------------------------------------------------------

