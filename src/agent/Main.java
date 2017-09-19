package agent;

import problem.ProblemSpec;

import java.io.IOException;

public class Main {

    public static void main(String args[]) throws IOException {
        ProblemSpec spec = new ProblemSpec();
        spec.loadProblem(args[0]);
        Sampler sampler = new Sampler(spec);
        sampler.sampleConfigSpace();








    }
}
