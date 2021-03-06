package agent;

public class WeightedSamplingStrategy {
    SamplingStrategy strat;
    double p=0;
    double weight;
    public WeightedSamplingStrategy(SamplingStrategy strat, double w){
        this.strat = strat;
        this.weight = w;
    }
    public SamplingStrategy getStrat() {
        return strat;
    }
    public double getWeight() {
        return weight;
    }
    public void setStrat(SamplingStrategy strat) {
        this.strat = strat;
    }
    public void setWeight(double weight) {
        this.weight = weight;
    }
    public double getProb() {
        return p;
    }
    public void setProb(double p) {
        this.p = p;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        WeightedSamplingStrategy that = (WeightedSamplingStrategy) o;

        return strat == that.strat;
    }

    @Override
    public int hashCode() {
        return strat.hashCode();
    }
}
