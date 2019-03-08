namespace gr_map_utils{
    class MapConverterInterface{
        public:
            virtual bool storeMap() = 0;
            virtual bool getMap() = 0;
            virtual ~MapConverterInterface(){}
    };
};
