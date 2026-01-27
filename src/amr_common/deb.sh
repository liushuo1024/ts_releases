#!/bin/bash
deb_pkg()
{
  bloom-generate rosdebian --os-name ubuntu --ros-distro noetic
  sed -i 's/\tdh_auto_build/\tdh_auto_build --parallel/'  debian/rules
  sed -i '/export DEB_CXXFLAGS_MAINT_APPEND=-DNDEBUG/a export DEB_BUILD_OPTIONS=parallel=4' debian/rules
  sed -i '/dh_shlibdeps -l/c dh_shlibdeps --dpkg-shlibdeps-params=--ignore-missing-info'  debian/rules
  fakeroot debian/rules binary
  rm -rf ./debian ./.obj-x86_64-linux-gnu
}

deb_pkg
rm ../*.ddeb
mv ../ros-noetic-amr-common* ~/deb