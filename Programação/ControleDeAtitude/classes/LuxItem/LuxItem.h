#ifndef LUXITEM_H
#define LUXITEM_H

#define NULL_LUXITEM -1

class LuxItem {
    public:
        LuxItem(int iluminancia, int angulo);
        LuxItem();
        int iluminancia;
        int angulo;
    private:
};

#endif
