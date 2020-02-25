#pragma once
#include"myFunction.h"
namespace hlg
{
    class Thing
    {
    public:
        Thing(int id, const Rect &detected_box);
        unsigned int track_frame;//����ƥ��ɹ�֡�����������һ����Ŀ�����ж�Ϊ����
        int id;//id��
        bool confidenceDecrease();
        void confidenceIncrease();
        Rect box;//��Ӿ��ο�
    private:
        double confidence;//���Ŷ�
        int confidence_inc;
        int confidence_dec;
    };
    class ThingTracker//�������ж�������������˸��˸о�û�б�Ҫʹ��kalman�����˶�����ͽ�����������ʰ�kalmanȥ���ˣ�������ƥ����Ȼ����
    {
    public:
        ThingTracker(string thing_name = "th");
        void track(const vector<Rect>&Thing_Detected, const Rect &Thing_ROI);
        vector<Thing> tracking_things;
    

    private:
        string name;
        vector<bool> idTabel;
        void idTabelDelete(int id);
        int idCreator();
    };
}