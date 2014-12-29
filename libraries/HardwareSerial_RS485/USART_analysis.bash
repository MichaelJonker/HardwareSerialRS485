unset __avr__

#__avr__="`find /Arduino/ -readable -type d -path "*/arduino-*/hardware/tools/avr/avr/include/avr"`"

anal_dir="`pwd`"

pushd /Arduino/arduino-1.0.5/hardware/tools/avr/avr/include/avr

grep -H -E -e "include[[:space:]]+(<|\")avr/io"                                                              io*.h | sed -e "s/:#[[:space:]]*include/ \t/" -e "s/\/\*.*$//"  >>"$anal_dir"/USART_analysis.txt
grep -H -E -e "define[[:space:]]+(UCSR|UBRR[0-9]*[[:space:]])"                                               io*.h | SED -e "s/:#[[:space:]]*define/  \t/" -e "s/\/\*.*$//"  >>"$anal_dir"/USART_analysis.txt
grep -H -E -e "define[[:space:]]+USART.*_vect[[:space:]]"                                                    io*.h | SED -e "s/:#[[:space:]]*define/  \t/" -e "s/\/\*.*$//"  >>"$anal_dir"/USART_analysis.txt
grep -H -E -e "define[[:space:]]+(UDR|MPCM|U2X|UPE|DOR|FE|TXC|TXEN|RXEN|UDRIE|TXCIE|RXCIE)[0-9]*[[:space:]]" io*.h | SED -e "s/:#[[:space:]]*define/  \t/" -e "s/\/\*.*$//"  >>"$anal_dir"/USART_analysis.txt
popd
