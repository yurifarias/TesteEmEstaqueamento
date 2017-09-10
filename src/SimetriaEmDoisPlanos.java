import Jama.Matrix;

public class SimetriaEmDoisPlanos extends AnalisesPreliminares {

    private double xoYCL;
    private double xoZBL;

    public Matrix reacoesNormais(Estaca[] estacas, double[][] esfExternos) {

        novasCoordenadas(estacas);

        Matrix reacoesNormais = mMovElasticoTransformado(estacas, esfExternos).transpose().times(mComponentesDeVetorTransformada(estacas));

        return reacoesNormais;
    }

    /* Método para achar as coordenadas dos novos eixos coordenados (xo, yo), em relação ao antigo,
    e o ângulo de rotação fi, após a translação e rotação dos antigos eixos coordenados como
    aritifício para solucionar casos de estaqueamento simétrico por um plano. */
    private void novasCoordenadas(Estaca[] estacas) {

        double[][] mS = mRigidez(estacas).getArray();

        xoYCL = mS[1][5] / mS[1][1];
        xoZBL = -mS[2][4] / mS[2][2];

    }

    private Matrix tensorTransformacao() {

        double[][] tensorT = new double[6][6];

        tensorT[0][0] = 1;
        tensorT[0][1] = 0;
        tensorT[0][2] = 0;
        tensorT[0][3] = 0;
        tensorT[0][4] = 0;
        tensorT[0][5] = 0;
        tensorT[1][0] = 0;
        tensorT[1][1] = 1;
        tensorT[1][2] = 0;
        tensorT[1][3] = 0;
        tensorT[1][4] = 0;
        tensorT[1][5] = 0;
        tensorT[2][0] = 0;
        tensorT[2][1] = 0;
        tensorT[2][2] = 1;
        tensorT[2][3] = 0;
        tensorT[2][4] = 0;
        tensorT[2][5] = 0;
        tensorT[3][0] = 0;
        tensorT[3][1] = 0;
        tensorT[3][2] = 0;
        tensorT[3][3] = tensorT[0][0];
        tensorT[3][4] = tensorT[0][1];
        tensorT[3][5] = tensorT[0][2];
        tensorT[4][0] = 0;
        tensorT[4][1] = 0;
        tensorT[4][2] = xoZBL;
        tensorT[4][3] = tensorT[1][0];
        tensorT[4][4] = tensorT[1][1];
        tensorT[4][5] = tensorT[1][2];
        tensorT[5][0] = 0;
        tensorT[5][1] = -xoYCL;
        tensorT[5][2] = 0;
        tensorT[5][3] = tensorT[2][0];
        tensorT[5][4] = tensorT[2][1];
        tensorT[5][5] = tensorT[2][2];

        return new Matrix(tensorT);

    }

    private Matrix mRigidezTransformada(Estaca[] estacas) {

        Matrix mRigidezTransformada = tensorTransformacao().times(mRigidez(estacas)).times(tensorTransformacao().transpose());
        return mRigidezTransformada;
    }

    private Matrix esfExternosTransformados(double[][] esfExternos) {

        Matrix mEsfExternosTransformados = tensorTransformacao().times(new Matrix(esfExternos));
        return mEsfExternosTransformados;
    }

    private Matrix mMovElasticoTransformado(Estaca[] estacas, double[][] esfExternos) {

        Matrix mMovimentoElasticoTranformado = mRigidezTransformada(estacas).solve(esfExternosTransformados(esfExternos));
        return mMovimentoElasticoTranformado;

        }

    private Matrix mComponentesDeVetorTransformada(Estaca[] estacas) {

        Matrix mComponentesDeVetorTransformada = tensorTransformacao().times(mComponentesDeVetor(estacas));
        return mComponentesDeVetorTransformada;

        }

}
