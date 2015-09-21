if [ $1 == 'bob' ]; then
    echo 'bob.ltl'
    java -ea -Xmx512m -cp /home/catherine/LTLMoP/src/etc/jtlv/jtlv-prompt1.4.0.jar:/home/catherine/LTLMoP/src/etc/jtlv/GROne GROneMain /home/catherine/LTLMoP/src/examples/two_robot_negotiation/bob/bob.smv /home/catherine/LTLMoP/src/examples/two_robot_negotiation/bob/bob.ltl 
elif [ $1 == 'alice' ]; then
    echo 'alice.ltl'
    java -ea -Xmx512m -cp /home/catherine/LTLMoP/src/etc/jtlv/jtlv-prompt1.4.0.jar:/home/catherine/LTLMoP/src/etc/jtlv/GROne GROneMain /home/catherine/LTLMoP/src/examples/two_robot_negotiation/alice/alice.smv /home/catherine/LTLMoP/src/examples/two_robot_negotiation/alice/alice.ltl  
elif [ $1 == 'alice_wander' ]; then
    echo 'alice_wander.ltl'
    java -ea -Xmx512m -cp /home/catherine/LTLMoP/src/etc/jtlv/jtlv-prompt1.4.0.jar:/home/catherine/LTLMoP/src/etc/jtlv/GROne GROneMain /home/catherine/LTLMoP/src/examples/two_robot_negotiation/city/wander/alice/alice_wander.smv  /home/catherine/LTLMoP/src/examples/two_robot_negotiation/city/wander/alice/alice_wander.ltl  
elif [ $1 == 'bob_wander' ]; then
    echo 'bob_wander.ltl'
    java -ea -Xmx512m -cp /home/catherine/LTLMoP/src/etc/jtlv/jtlv-prompt1.4.0.jar:/home/catherine/LTLMoP/src/etc/jtlv/GROne GROneMain /home/catherine/LTLMoP/src/examples/two_robot_negotiation/city/wander/bob/bob_wander.smv  /home/catherine/LTLMoP/src/examples/two_robot_negotiation/city/wander/bob/bob_wander.ltl  
    
else 
    echo 'please specify either bob or alice'
fi
