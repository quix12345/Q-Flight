

public class PID
{
    private float Kp;
    private float Kd;
    private float Ki;
    private float error = 0;
    private float prevError = 0;
    public float integ = 0;
    private float  deriv = 0;
    private float desired = 0;
    public float outP;       
    public float outI;      
    public float outD;        
    private float iLimit;   

    public PID(float Kp,float Kd,float Ki)
    {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
    }

    public float PIDupdate(float measured, float expect)
    {
        float output;

        this.desired = expect;                        

        this.error = this.desired - measured;         

        this.integ += this.error;

        if (this.integ > this.iLimit)          
        {
            this.integ = this.iLimit;
        }
        else if (this.integ < -this.iLimit)    
        {
            this.integ = -this.iLimit;
        }
        this.deriv = this.error - this.prevError;     

        this.outP = this.Kp * this.error;           
        this.outI = this.Ki * this.integ;
        this.outD = this.Kd * this.deriv;

        output = this.outP +
                         this.outI +
                         this.outD;

        this.prevError = this.error;                               

        return output;
    }

    public void pidSetIntegralLimit(float limit)
    {
        this.iLimit = limit;
    }
}
