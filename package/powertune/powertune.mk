################################################################################
#
# powertune
#
################################################################################

POWERTUNE_VERSION = origin/master
POWERTUNE_SITE = git://github.com/BastianGschrey/PowerTune
POWERTUNE_LICENSE = MIT

POWERTUNE_DEPENDENCIES = qt5base qt5serialport qt5serialbus qt5quickcontrols qt5quickcontrols2 qt5declarative

define POWERTUNE_CONFIGURE_CMDS
	cd $(@D); $(TARGET_MAKE_ENV) $(HOST_DIR)/usr/bin/qmake -makefile
endef

define POWERTUNE_BUILD_CMDS
	$(TARGET_MAKE_ENV) $(MAKE) -C $(@D)
endef

define POWERTUNE_INSTALL_TARGET_CMDS
	$(INSTALL) -D -m 0755 $(@D)/PowertuneQMLGui $(TARGET_DIR)/usr/bin 
endef

$(eval $(generic-package))
