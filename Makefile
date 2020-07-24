# Create by wuying, 2013/04/15

ifeq ($(PARAM_FILE), )
    PARAM_FILE:=../../mpp/Makefile.param
    include $(PARAM_FILE)
endif

obj-m := irqkey.o
hiuser-y += irqkey.o

all:
	@echo -e "\e[0;32;1m--Compiling 'irqkey'...\e[0;36;1m" 
	@echo -e "\e[0m" 
	@make -C $(LINUX_ROOT) M=$(PWD) modules
	@cp irqkey.ko $(REL_KO)

clean: 
	@make -C $(LINUX_ROOT) M=$(PWD) clean 
	@rm -rf $(REL_KO)/irqkey.ko
