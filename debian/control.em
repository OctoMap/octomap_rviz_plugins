Source: @(Package)
Section: misc
Priority: extra
Maintainer: @(Maintainer)
Build-Depends: debhelper (>= @(debhelper_version).0.0), @(', '.join(BuildDepends))
Homepage: @(Homepage)
Standards-Version: 3.9.2

Package: @(Package)
Architecture: any
Depends: ${shlibs:Depends}, ${misc:Depends}, @(', '.join(Depends))
Description: @(Description)
