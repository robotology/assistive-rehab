#!/bin/bash

speed() {
    Q_1="A che velocita devo andare"
    Q_2="Quanto devo andare veloce"
    Q_3="Che andatura devo avere"  
    NUMBER=$[ ( $RANDOM % 3 )  + 1 ]
    Q=Q_${NUMBER}
    printf "\nAsking : ${!Q}\n"
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
}

aid() {
    Q_1="Posso usare il mio bastone"
    Q_2="Posso usare il deambulatore"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    printf "\nAsking : ${!Q}\n"
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
}

repetition() {
    Q_1="Quante volte devo ripetere l'esercizio"
    Q_2="Quante ripetizioni devo fare"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    printf "\nAsking : ${!Q}\n"
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
}

feedback() {
    Q_1="Come sto facendo l'esercizio"
    Q_2="Sto andando bene"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    printf "\nAsking : ${!Q}\n"
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
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
