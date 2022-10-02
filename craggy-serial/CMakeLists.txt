cmake_minimum_required(VERSION 3.17)
project(roughtime C)

set(CMAKE_C_STANDARD 11)

option(CRAGGY_WITH_UDP_TRANSPORT "Enable UDP transport" ON)
option(CRAGGY_WITH_OPENSSL_BINDINGS "Use OpenSSL cryptographic operations" OFF)
option(CRAGGY_WITH_ORLP_ED25519_BINDINGS "Use ORLPs ED25519 cryptographic operations" OFF)

add_subdirectory(library)

add_subdirectory(roughtest)


#enable_testing()
#add_subdirectory(test)
