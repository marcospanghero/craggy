project(roughtest C)

set(SOURCES
        base64 ublox.c rtkcmn.c rcvraw.c sbas.c serial-driver.c
        main)

add_executable(roughtest ${SOURCES})
target_link_libraries(roughtest craggy m)

if (CRAGGY_WITH_OPENSSL_BINDINGS)
    find_package(OpenSSL REQUIRED)
    target_link_libraries(roughtest OpenSSL::SSL)
endif()
