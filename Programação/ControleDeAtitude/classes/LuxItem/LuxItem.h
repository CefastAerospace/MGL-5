#ifndef LUXITEM_H
#define LUXITEM_H

#define NULL_LUXITEM -1

class LuxItem {
    public:
        // Métodos públicos
        LuxItem(int iluminancia, int angulo);
        LuxItem();

        // Atributos públicos
        int iluminancia;
        int angulo;
    private:
};

#endif
