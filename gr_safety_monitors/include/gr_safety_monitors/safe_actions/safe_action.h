
namespace gr_safety_monitors{
    class SafeAction{
        public:
            virtual ~SafeAction()=0;
        protected:
            virtual void execute()=0;
    };
};