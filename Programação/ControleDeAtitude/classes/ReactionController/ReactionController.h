#ifndef ReactionController_H
#define ReactionController_H

#include <Arduino.h>
#include "LuxList/LuxList.h"
#include "BrushlessMotor/BrushlessMotor.h"

class ReactionController {
    public:
        ReactionController(BrushlessMotor motor);
        void leituraAmbiente();
        void estabiliza(int angulo);

    private:
        BrushlessMotor motor;
        LuxList luxList;
        LuxItem luxAtual;
        
        void atualizaLuxAtual();
        int corrigeAngulo(int angulo);
    
};

#endif