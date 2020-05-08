#!/bin/bash

speed() {
    Q_1="A che velocita devo andare"
    Q_2="Quanto devo andare veloce"
    Q_3="Che andatura devo avere"  
    NUMBER=$[ ( $RANDOM % 3 )  + 1 ]
    Q=Q_${NUMBER}
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
    echo "$(tput setab 1)$(tput setaf 7)Asking: ${!Q}$(tput sgr0)"
}

aid() {
    Q_1="Posso usare il mio bastone"
    Q_2="Posso usare il deambulatore"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
    echo "$(tput setab 1)$(tput setaf 7)Asking: ${!Q}$(tput sgr0)"
}

repetition() {
    Q_1="Quante volte devo ripetere l'esercizio"
    Q_2="Quante ripetizioni devo fare"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
    echo "$(tput setab 1)$(tput setaf 7)Asking: ${!Q}$(tput sgr0)"
}

feedback() {
    Q_1="Come sto facendo l'esercizio"
    Q_2="Sto andando bene"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
    echo "$(tput setab 1)$(tput setaf 7)Asking: ${!Q}$(tput sgr0)"
}

#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
userinput=""
while read key
do
  #if [ $key != "$speed"]
  #  then
  #  printf "\nCan't ask this one\n"
  #fi
  $key
done


$1
