#!/bin/bash

speed() {
    Q_1="A che velocita devo andare"
    Q_2="Quanto devo andare veloce"
    Q_3="Che andatura devo avere"
    echo "List of possible questions: ${Q_1}, ${Q_2}, ${Q_3}"
    NUMBER=$[ ( $RANDOM % 3 )  + 1 ]
    Q=Q_${NUMBER}
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
    echo "$(tput setab 1)$(tput setaf 7)Asking: ${!Q}$(tput sgr0)"
}

aid() {
    Q_1="Posso usare il mio bastone"
    Q_2="Posso usare il deambulatore"
    echo "List of possible questions: ${Q_1}, ${Q_2}"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
    echo "$(tput setab 1)$(tput setaf 7)Asking: ${!Q}$(tput sgr0)"
}

repetition() {
    Q_1="Quante volte devo ripetere l'esercizio"
    Q_2="Quante ripetizioni devo fare"
    echo "List of possible questions: ${Q_1}, ${Q_2}"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
    echo "$(tput setab 1)$(tput setaf 7)Asking: ${!Q}$(tput sgr0)"
}

feedback() {
    Q_1="Come sto facendo l'esercizio"
    Q_2="Sto andando bene"
    echo "List of possible questions: ${Q_1}, ${Q_2}"
    NUMBER=$[ ( $RANDOM % 2 )  + 1 ]
    Q=Q_${NUMBER}
    echo ${!Q} | yarp write ... /googleSpeechProcess/text:i
    echo "$(tput setab 1)$(tput setaf 7)Asking: ${!Q}$(tput sgr0)"
}

#######################################################################################
# "MAIN" FUNCTION:                                                                    #
#######################################################################################
while read key
do
  $key
done


$1
