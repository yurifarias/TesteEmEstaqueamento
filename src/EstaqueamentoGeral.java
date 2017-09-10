import Jama.Matrix;

public class EstaqueamentoGeral extends AnalisesPreliminares {

    public Matrix movElastico(Estaca[] estacas, double[][] esfExternos) {

        return mRigidez(estacas).solve(new Matrix(esfExternos));
    }

    public Matrix reacoesNormais(Estaca[] estacas, double[][] esfExternos) {

        return movElastico(estacas,esfExternos).transpose().times(mComponentesDeVetor(estacas));
    }

}
