SECTIONS
{
  /* Create space for a hash.  Currently an MD5 has is used, which is 16   */
  /* bytes long. however the last 64 bytes are RESERVED for hash related   */
  .exst_hash :
  {
    /* 64 bytes is the size of an MD5 hashing block size.                  */
    . = ORIGIN(EXST_HASH);

    BYTE(0x00); /* block format */
    BYTE(0x00); /* Checksum method, 0x00 = MD5 hash */
    BYTE(0x00); /* Reserved */
    BYTE(0x00); /* Reserved */
    
    /* Fill the last 60 bytes with data, including an empty hash aligned   */
    
    /* to the last 16 bytes.                                               */
    FILL(0x00000000); /* Reserved */
    
    . = ORIGIN(EXST_HASH) + LENGTH(EXST_HASH) - 16;
    __md5_hash_address__ = .;
    LONG(0x00000000);
    LONG(0x00000000);
    LONG(0x00000000);
    LONG(0x00000000);
    . = ORIGIN(EXST_HASH) + LENGTH(EXST_HASH);
    __firmware_end__ = .;
  } >EXST_HASH
}