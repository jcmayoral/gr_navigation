namespace gr_safety_monitors{
    class SafeAction{
        public:
            virtual ~SafeAction(){};
            virtual void execute()=0;
    };
};