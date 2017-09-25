package agent;
import java.util.ArrayList;

/***
 * Data structure made to hold a list of sampling strategy
 * provides the helper function getSumOfWeight which does what it names entails
 */
public class SamplingStrategyList extends ArrayList<WeightedSamplingStrategy>{
    private static final long serialVersionUID = 251726265965715745L;

    public SamplingStrategyList(){
        super();
    }

    /**
     *
     * @return Sum of all weights of strategies in list
     */
    public int getSumOfWeight(){
        int retval=0;
        for(WeightedSamplingStrategy s : this){
            retval+=s.getWeight();
        }
        return retval;
    }
}
